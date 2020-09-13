/******************************************************************************
 * File:             mapper.cpp
 *
 * Author:           Akash Sharma
 * Created:          06/26/20
 * Description:      Mapper thread implementation
 *****************************************************************************/
#include "mapper.h"

#include <Cuda/OSLAMUtils/SegmentationCuda.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <utility>

#include "object-slam/struct/instance_image.h"
#include "object-slam/utils/utils.h"

namespace oslam
{
    Mapper::Mapper(Map::Ptr map,
                   TrackerOutputQueue* tracker_output_queue,
                   TransportOutputQueue* transport_output_queue,
                   OutputQueue* output_queue)
        : MISO(output_queue, "Mapper"),
          map_(map),
          tracker_output_queue_(tracker_output_queue),
          transport_output_queue_(transport_output_queue)
    {
    }

    Mapper::InputUniquePtr Mapper::getInputPacket()
    {
        auto start_time = Timer::tic();

        TrackerOutput::UniquePtr mapper_payload;
        bool queue_state = tracker_output_queue_->popBlocking(mapper_payload);
        if (!mapper_payload || !queue_state)
        {
            spdlog::error("Module: {} {} returned null", name_id_, tracker_output_queue_->queue_id_);
            return nullptr;
        }
        curr_timestamp_                 = mapper_payload->timestamp_;
        const bool is_current_maskframe = mapper_payload->frame_.is_keyframe_;

        Mapper::InputUniquePtr mapper_input;
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            mapper_input = std::make_unique<MapperInput>(curr_timestamp_,
                                                         keyframe_timestamps_.back(),
                                                         mapper_payload->tracker_status_,
                                                         mapper_payload->frame_,
                                                         prev_transport_output_->instance_images_,
                                                         mapper_payload->relative_camera_pose_,
                                                         mapper_payload->information_matrix_);
        }
        else
        {
            ImageTransportOutput::UniquePtr transport_output;
            // Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            // current frame
            if (!syncQueue<ImageTransportOutput::UniquePtr>(curr_timestamp_, transport_output_queue_, &transport_output))
            {
                spdlog::error("Missing masked image with requested timestamp: {}", curr_timestamp_);
                return nullptr;
            }
            if (!transport_output)
            {
                spdlog::error("Module: {} {} returned null", name_id_, transport_output_queue_->queue_id_);
                return nullptr;
            }
            mapper_input = std::make_unique<MapperInput>(curr_timestamp_,
                                                         curr_timestamp_,
                                                         mapper_payload->tracker_status_,
                                                         mapper_payload->frame_,
                                                         transport_output->instance_images_,
                                                         mapper_payload->relative_camera_pose_,
                                                         mapper_payload->information_matrix_);

            prev_transport_output_ = std::move(transport_output);
            keyframe_timestamps_.push_back(curr_timestamp_);
        }
        if (!mapper_input)
        {
            spdlog::error("Unable to create MapperInput");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::debug("Processed Mapper payload: {}, took {} ms", curr_timestamp_, duration);
        return mapper_input;
    }

    Mapper::OutputUniquePtr Mapper::runOnce(Mapper::InputUniquePtr input)
    {
        using namespace gtsam;

        const MapperInput& mapper_payload           = *input;
        curr_timestamp_                             = mapper_payload.timestamp_;
        const Timestamp& prev_keyframe_timestamp    = keyframe_timestamps_.back();
        const Frame& frame                          = mapper_payload.frame_;
        const InstanceImages& instance_images       = mapper_payload.instance_images_;
        const Eigen::Matrix4d& relative_camera_pose = mapper_payload.relative_camera_pose_;
        MapperStatus mapper_status                  = MapperStatus::INVALID;
        ObjectRendersUniquePtr object_renders       = std::make_unique<ObjectRenders>();

        if (curr_timestamp_ == 1)
        {
            // Instantiate objects and background in map
            if (mapper_payload.tracker_status_ == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d& camera_pose = relative_camera_pose;
                T_camera_to_world_trajectory_.push_back(camera_pose);

                // Add camera pose and factor to the graph
                TSDFObject::Ptr bg = createBackground(frame, camera_pose);
                active_bg_id_      = bg->id_;

                auto camera_key = gtsam::Symbol('c', curr_timestamp_);

                // Prior noise of 0.01 metres (x,y,z) and 0.01 rad in (alpha, beta, gamma)
                auto prior_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.01));

                auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));

                //! prior --> camera <--> background
                object_pose_graph_.emplace_shared<PriorFactor<Pose3>>(camera_key, Pose3(camera_pose), prior_noise);
                object_pose_values_.insert(camera_key, Pose3(camera_pose));

                spdlog::debug("Created background object and emplaced in graph");
                map_->addObject(bg, true);

                // Add each object in the first frame as a landmark
                for (const auto& instance_image : instance_images)
                {
                    TSDFObject::Ptr object = createObject(frame, instance_image, relative_camera_pose);
                    if (!object)
                    {
                        spdlog::debug("Object construction failed!");
                        continue;
                    }

                    auto object_key = object->hash(object->id_);

                    // Initially we estimate some empirical noise value
                    spdlog::debug("Current object pose:\n {}", object->getPose());

                    auto T_object_to_camera = camera_pose.inverse() * object->getPose();
                    object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                        camera_key, object_key, Pose3(T_object_to_camera), between_noise);

                    object_pose_values_.insert(object_key, Pose3(object->getPose()));

                    map_->addObject(object);
                }
                renderMapObjects(object_renders, frame, camera_pose);
                mapper_status = MapperStatus::VALID;
            }
        }
        else
        {
            if (mapper_payload.tracker_status_ == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d camera_pose = T_camera_to_world_trajectory_.back() * relative_camera_pose;
                T_camera_to_world_trajectory_.push_back(camera_pose);

                if (curr_timestamp_ == prev_keyframe_timestamp && instance_images.size() >= 2)
                {
                    auto camera_key                   = gtsam::Symbol('c', curr_timestamp_);
                    spdlog::info("Current camera key: {}", camera_key);
                    auto prev_prev_keyframe_timestamp = *(keyframe_timestamps_.end()-2);
                    spdlog::info("prev_prev_keyframe_timestamp: {}", prev_prev_keyframe_timestamp);
                    auto prev_camera_key              = gtsam::Symbol('c', prev_prev_keyframe_timestamp);
                    spdlog::info("prev_camera_key: {}", prev_camera_key);
                    auto T_camera_to_prev_keyframe =
                        T_camera_to_world_trajectory_.at(prev_prev_keyframe_timestamp).inverse() * camera_pose;
                    spdlog::info("Relative pose: {}", T_camera_to_prev_keyframe);
                    auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));

                    //! TODO: Add camera poses to the posegraph
                    object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                        camera_key, prev_camera_key, Pose3(T_camera_to_prev_keyframe), between_noise);
                    object_pose_values_.insert(camera_key, Pose3(camera_pose));
                }

                TSDFObject::Ptr active_bg = map_->getBackground();
                active_bg->integrate(frame, InstanceImage(frame.width_, frame.height_), camera_pose);

                //! TODO: Raycast only those objects in the camera frustum
                renderMapObjects(object_renders, frame, camera_pose);

                // Project the masks to current frame
                auto T_keyframe_to_world             = T_camera_to_world_trajectory_.at(prev_keyframe_timestamp - 1);
                auto T_camera_to_world               = T_camera_to_world_trajectory_.back();
                Eigen::Matrix4d T_keyframe_to_camera = T_camera_to_world.inverse() * T_keyframe_to_world;
                cuda::TransformCuda T_keyframe_to_camera_cuda;
                T_keyframe_to_camera_cuda.FromEigen(T_keyframe_to_camera);
                cuda::PinholeCameraIntrinsicCuda intrinsic_cuda(frame.intrinsic_);

                InstanceImages frame_instance_images;
                std::vector<bool> frame_instance_matches;
                frame_instance_images.reserve(instance_images.size());
                frame_instance_matches.reserve(instance_images.size());
                for (const auto& instance_image : instance_images)
                {
                    //! Skip for background object
                    if (instance_image.label_ == 0)
                    {
                        continue;
                    }
                    //! Project the mask from maskframe to current frame
                    cuda::ImageCuda<uchar, 1> src_mask;
                    src_mask.Upload(instance_image.maskb_);

                    cv::Mat depthf;
                    cv::rgbd::rescaleDepth(frame.depth_, CV_32F, depthf);
                    cuda::ImageCuda<float, 1> depth;
                    depth.Upload(depthf);

                    // TODO: Use geometric segmentation
                    auto proj_mask_cuda = cuda::SegmentationCuda::TransformAndProject(
                        src_mask, depth, T_keyframe_to_camera_cuda, intrinsic_cuda);
                    cv::Mat proj_mask = proj_mask_cuda.DownloadMat();

                    BoundingBox proj_bbox;
                    transform_project_bbox(instance_image.bbox_, proj_bbox, depthf, frame.intrinsic_, T_keyframe_to_camera);
                    frame_instance_images.emplace_back(proj_mask, proj_bbox, instance_image.label_, instance_image.score_);
                    frame_instance_matches.emplace_back(false);
                }

                for (const auto& object_pair : *object_renders)
                {
                    const ObjectId& id                  = object_pair.first;
                    const ObjectRender& object_render   = object_pair.second;
                    const cv::Mat& object_raycast_color = object_render.color_map_;
                    TSDFObject::Ptr object              = map_->getObject(id);

                    if (object->isBackground())
                        continue;

                    auto iter = associateObjects(id, object_raycast_color, frame_instance_images, frame_instance_matches);

                    if (iter == frame_instance_images.end())
                    {
                        //! Increment the non-existence count
                        object->non_existence_++;
                    }
                    else
                    {
                        object->existence_++;
                        object->integrate(frame, *iter, camera_pose);

                        if (curr_timestamp_ == prev_keyframe_timestamp && instance_images.size() >= 2)
                        {
                            auto camera_key         = gtsam::Symbol('c', prev_keyframe_timestamp);
                            auto object_key         = object->hash(id);
                            auto T_object_to_camera = camera_pose.inverse() * object->getPose();
                            //! TODO: Add correct variances for heaven's sake!!!
                            auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));
                            object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                                camera_key, object_key, Pose3(T_object_to_camera), between_noise);
                        }
                    }
                    spdlog::debug("Object existence: {}, non-existence: {}", object->existence_, object->non_existence_);
                }

                spdlog::info("Number of frame instance matches: {}", frame_instance_matches.size());

                // Create new objects for unmatched instance images
                for (size_t i = 0; i < frame_instance_matches.size(); i++)
                {
                    if (!frame_instance_matches.at(i))
                    {
                        spdlog::info("Creating object: {}", i);
                        spdlog::info("Camera pose: {}", camera_pose);
                        spdlog::info("frame_instance_images size: {}, frame_instance_matches size: {}",
                                     frame_instance_images.size(),
                                     frame_instance_matches.size());
                        spdlog::info("instance image score: {}", frame_instance_images.at(i).score_);
                        spdlog::info("mask size: {}", cv::countNonZero(frame_instance_images.at(i).maskb_));
                        auto object = createObject(frame, frame_instance_images.at(i), camera_pose);
                        spdlog::info("Created object: {}", i);
                        if (!object)
                        {
                            spdlog::debug("Object creation failed!");
                            continue;
                        }
                        auto object_key = object->hash(object->id_);
                        //! Check: Should return the same camera key
                        auto camera_key = gtsam::Symbol('c', prev_keyframe_timestamp);

                        auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));
                        //! TODO: Some asserts for invalid maskframe_timestamps?
                        auto T_object_to_keyframe = T_keyframe_to_world.inverse() * object->getPose();

                        //! Relative pose with previous keyframe (but since background is identity)
                        object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                            camera_key, object_key, Pose3(T_object_to_keyframe), between_noise);

                        object_pose_values_.insert(object_key, Pose3(object->getPose()));

                        map_->addObject(object);
                        spdlog::info("Added object into the map");
                    }
                    else
                    {
                        spdlog::debug("Object instance {} matched", i);
                    }
                }

#ifdef OSLAM_DEBUG_VIS
                object_pose_graph_.print("Factor Graph:\n");
                object_pose_values_.print("Values to optimize: \n");
#endif
                // Evaluate remove objects with very low existence probability
                std::vector<ObjectId> to_delete_objects;
                for (const auto& object_pair : map_->id_to_object_)
                {
                    const ObjectId id           = object_pair.first;
                    TSDFObject::ConstPtr object = object_pair.second;

                    if (object->isBackground())
                    {
                        continue;
                    }

                    double existence_expect = object->getExistExpectation();
                    spdlog::debug("{} -> Existence expectation: {}", id, existence_expect);

                    if (existence_expect < 0.2)
                    {
                        to_delete_objects.push_back(id);
                    }
                }

                for (const auto& object_id : to_delete_objects)
                {
                    spdlog::info("Removing object {}", object_id);
                    map_->removeObject(object_id);
                }

                if (shouldCreateNewBackground(frame.timestamp_))
                {
                    TSDFObject::Ptr new_bg = createBackground(frame, camera_pose);
                    map_->addObject(new_bg, true);
                    spdlog::info("Added new background object into the map");
                }

                if (prev_keyframe_timestamp == curr_timestamp_)
                {
                    std::unique_ptr<gtsam::LevenbergMarquardtOptimizer> optimizer =
                        std::make_unique<gtsam::LevenbergMarquardtOptimizer>(object_pose_graph_, object_pose_values_);
                    gtsam::Values new_values = optimizer->optimize();
                    spdlog::info("Graph error before optimize: {}", object_pose_graph_.error(object_pose_values_));
                    spdlog::info("Graph error after optimize: {}", object_pose_graph_.error(new_values));
                    updateMap(new_values);

                    //! Create new background here with updated keyframe_timestamp_ pose
                    spdlog::info("Optimized latest pose: {}", T_camera_to_world_trajectory_.back());
                    TSDFObject::Ptr new_bg =
                        createBackground(frame, T_camera_to_world_trajectory_.back());
                    map_->addObject(new_bg, true);
                }
            }
            mapper_status = MapperStatus::VALID;
        }
        return std::make_unique<RendererInput>(
            curr_timestamp_, mapper_status, std::move(object_renders), frame, T_camera_to_world_trajectory_);
    }

    TSDFObject::Ptr Mapper::createBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose)
    {
        using namespace open3d;
        // Create default instance image for background
        InstanceImage background_instance(frame.width_, frame.height_);
        ObjectId background_id(0, frame.timestamp_, BoundingBox({ 0, 0, frame.width_ - 1, frame.height_ - 1 }));
        TSDFObject::Ptr background =
            std::make_shared<TSDFObject>(background_id, frame, background_instance, camera_pose, BACKGROUND_RESOLUTION);

        background->integrate(frame, background_instance, camera_pose);
        return background;
    }

    bool Mapper::shouldCreateNewBackground(Timestamp timestamp)
    {
        auto active_bg   = map_->getBackground();
        double vis_ratio = active_bg->getVisibilityRatio(timestamp);
        spdlog::debug("Visibility Ratio: {}", vis_ratio);

        if (vis_ratio < 0.2)
            return true;
        return false;
    }

    TSDFObject::Ptr Mapper::createObject(const Frame& frame,
                                         const InstanceImage& instance_image,
                                         const Eigen::Matrix4d& camera_pose)
    {
        if (instance_image.score_ < SCORE_THRESHOLD)
        {
            spdlog::warn(
                "Object {} with score {} is below score threshold. Not added", instance_image.label_, instance_image.score_);
            return nullptr;
        }
        // If the masksize is smaller that 50^2 pixels
        if (cv::countNonZero(instance_image.maskb_) < 2500)
        {
            spdlog::warn("Object {} width score {} is too small. Not added", instance_image.label_, instance_image.score_);
            return nullptr;
        }

        Eigen::Vector2i object_center = Eigen::Vector2i((instance_image.bbox_[0] + instance_image.bbox_[2]) / 2,
                                                        (instance_image.bbox_[1] + instance_image.bbox_[3]) / 2);
        spdlog::info("Object center: {}", object_center);
        if (!(object_center[0] >= InstanceImage::BORDER_WIDTH &&
              object_center[0] < frame.width_ - InstanceImage::BORDER_WIDTH &&
              object_center[1] >= InstanceImage::BORDER_WIDTH &&
              object_center[1] < frame.height_ - InstanceImage::BORDER_WIDTH))
        {
            spdlog::debug("Object near corner of the image, Object center: {}", object_center);
            return nullptr;
        }

        ObjectId object_id(instance_image.label_, frame.timestamp_, instance_image.bbox_);
        spdlog::info("Creating object ID: {}", object_id);
        TSDFObject::Ptr object =
            std::make_shared<TSDFObject>(object_id, frame, instance_image, camera_pose, OBJECT_RESOLUTION);
        spdlog::info("Created object: {} Integrating now", object_id);
        object->integrate(frame, instance_image, camera_pose);
        spdlog::info("Created object: {} Integrated", object_id);
        return object;
    }

    void Mapper::renderMapObjects(ObjectRendersUniquePtr& object_renders,
                                  const Frame& frame,
                                  const Eigen::Matrix4d& camera_pose)
    {
        using namespace open3d;
        object_renders->reserve(map_->id_to_object_.size());
        for (auto& object_pair : map_->id_to_object_)
        {
            const ObjectId& id      = object_pair.first;
            TSDFObject::Ptr& object = object_pair.second;

            cuda::ImageCuda<float, 3> vertex;
            cuda::ImageCuda<float, 3> normal;
            cuda::ImageCuda<uchar, 3> color;

            vertex.Create(frame.width_, frame.height_);
            normal.Create(frame.width_, frame.height_);
            color.Create(frame.width_, frame.height_);

            object->raycast(vertex, normal, color, camera_pose);

            cv::Mat color_map  = color.DownloadMat();
            cv::Mat vertex_map = vertex.DownloadMat();
            cv::Mat normal_map = normal.DownloadMat();

            object_renders->emplace(id, ObjectRender(color_map, vertex_map, normal_map));
        }
    }

    InstanceImages::const_iterator Mapper::associateObjects(const ObjectId& id,
                                                            const cv::Mat& object_raycast,
                                                            const InstanceImages& instance_images,
                                                            std::vector<bool>& instance_matches)
    {
        cv::Mat gray;
        cv::cvtColor(object_raycast, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask;
        cv::threshold(gray, mask, 10, 255, cv::THRESH_BINARY);

        auto iter = instance_images.begin();
        for (; iter != instance_images.end(); ++iter)
        {
            if (iter->score_ < SCORE_THRESHOLD)
            {
                spdlog::warn("InstanceImage {} with score {} is below score threshold. Not matched with {}",
                             iter->label_,
                             iter->score_,
                             id);
                continue;
            }

            cv::Mat intersection_mask, union_mask;
            cv::bitwise_and(iter->maskb_, mask, intersection_mask);
            cv::bitwise_or(iter->maskb_, mask, union_mask);

            int union_val        = cv::countNonZero(union_mask);
            int intersection_val = cv::countNonZero(intersection_mask);

            auto quality = static_cast<float>(intersection_val) / static_cast<float>(union_val);

            size_t curr_idx = size_t(iter - instance_images.begin());
            spdlog::debug("{} -> Quality of the association: {}, Target label: {}", id, quality, iter->label_);
            if (!instance_matches.at(curr_idx) && quality > IOU_OVERLAP_THRESHOLD && union_val > MASK_AREA_THRESHOLD)
            {
                instance_matches.at(curr_idx) = true;
                return iter;
            }
        }
        return instance_images.end();
    }

    void Mapper::updateMap(const gtsam::Values& values)
    {
        for (auto& object_pair : map_->id_to_object_)
        {
            const ObjectId& id      = object_pair.first;
            TSDFObject::Ptr& object = object_pair.second;

            if (values.exists(object->hash(id)))
            {
                gtsam::Pose3 updatedPose = values.at<gtsam::Pose3>(object->hash(id));
                object->setPose(updatedPose.matrix());
            }
        }
        for (const auto& keyframe_timestamp : keyframe_timestamps_)
        {
            auto camera_key = gtsam::Symbol('c', keyframe_timestamp);
            if (values.exists(camera_key))
            {
                gtsam::Pose3 updatedPose                                 = values.at<gtsam::Pose3>(camera_key);
                T_camera_to_world_trajectory_.at(keyframe_timestamp - 1) = updatedPose.matrix();
            }
        }
    }

}  // namespace oslam
