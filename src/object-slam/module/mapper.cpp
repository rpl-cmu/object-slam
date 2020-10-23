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
#include <spdlog/spdlog.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
          transport_output_queue_(transport_output_queue),
          object_renders_queue_("ObjectRendersQueue")
    {
        spdlog::trace("CONSTRUCT: Mapper");
    }

    Mapper::InputUniquePtr Mapper::getInputPacket()
    {
        spdlog::trace("Mapper::getInputPacket()");
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

        ObjectRenders::UniquePtr object_renders;
        if (curr_timestamp_ == 1)
        {
            object_renders = std::make_unique<ObjectRenders>(curr_timestamp_, Renders({}));
        }
        else
        {
            if (!syncQueue<ObjectRenders::UniquePtr>(curr_timestamp_, &object_renders_queue_, &object_renders))
            {
                spdlog::error("Missing object renders with requested timestamp: {}", curr_timestamp_);
                return nullptr;
            }
        }
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            mapper_input = std::make_unique<MapperInput>(curr_timestamp_,
                                                         keyframe_timestamps_.back(),
                                                         mapper_payload->tracker_status_,
                                                         mapper_payload->frame_,
                                                         prev_transport_output_->instance_images_,
                                                         object_renders->renders_,
                                                         mapper_payload->relative_camera_pose_);
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
                                                         object_renders->renders_,
                                                         mapper_payload->relative_camera_pose_);

            prev_transport_output_ = std::move(transport_output);
            keyframe_timestamps_.push_back(curr_timestamp_);
        }
        if (!mapper_input)
        {
            spdlog::error("Unable to create MapperInput");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::info("Processed Mapper payload: {}, took {} ms", curr_timestamp_, duration);
        return mapper_input;
    }

    Mapper::OutputUniquePtr Mapper::runOnce(Mapper::InputUniquePtr input)
    {
        spdlog::trace("Mapper::runOnce()");
        const MapperInput& mapper_payload           = *input;
        curr_timestamp_                             = mapper_payload.timestamp_;
        const Timestamp& keyframe_timestamp         = keyframe_timestamps_.back();
        const Frame& frame                          = mapper_payload.frame_;
        const InstanceImages& instance_images       = mapper_payload.instance_images_;
        const Renders& object_renders               = mapper_payload.object_renders_;
        const Eigen::Matrix4d& relative_camera_pose = mapper_payload.relative_camera_pose_;
        MapperStatus mapper_status                  = MapperStatus::INVALID;
        InstanceImage bg_instance                   = InstanceImage(frame.width_, frame.height_);

        bool needs_optimization = false;

        auto start_time = Timer::tic();
        switch (mapper_payload.tracker_status_)
        {
            case TrackerStatus::VALID:
            {
                if (curr_timestamp_ == 1)
                {
                    initializeMapAndGraph(frame, instance_images, relative_camera_pose);
                    bg_instance   = createBgInstanceImage(frame, object_renders, instance_images);
                    mapper_status = MapperStatus::VALID;
                    break;
                }
                else
                {
                    const Eigen::Matrix4d camera_pose = map_->getCameraPose(curr_timestamp_ - 1) * relative_camera_pose;

                    //! Add to the map
                    map_->addCameraPose(camera_pose);
                    spdlog::trace("Added camera pose into map");
                    if (curr_timestamp_ == keyframe_timestamp && keyframe_timestamps_.size() >= 2)
                    {
                        auto prev_keyframe_timestamp   = *(keyframe_timestamps_.end() - 2);
                        auto prev_keyframe_pose        = map_->getCameraPose(prev_keyframe_timestamp);
                        auto T_camera_to_prev_keyframe = prev_keyframe_pose.inverse() * camera_pose;

                        addCameraCameraBetweenFactor(curr_timestamp_, prev_keyframe_timestamp, T_camera_to_prev_keyframe);
                        addCameraValue(curr_timestamp_, camera_pose);
                        spdlog::info("Added camera factor into pose_graph");

                        needs_optimization = true;
                    }

                    //! Integrate background
                    map_->integrateBackground(frame, InstanceImage(frame.width_, frame.height_), camera_pose);
                    spdlog::trace("Integrated background");

                    //! Project the instance images (object masks) into the current frame
                    InstanceImages frame_instance_images;
                    projectInstanceImages(keyframe_timestamp, frame, instance_images, frame_instance_images);
                    spdlog::trace("Projected instance images");

                    //! Associate and integrate objects in the map
                    std::vector<bool> frame_instance_matches =
                        integrateObjects(object_renders, frame_instance_images, frame, camera_pose);
                    spdlog::trace("Integrated objects");

                    spdlog::trace("Number of frame instance matches: {}", frame_instance_matches.size());

                    // Create new objects for unmatched instance images
                    bool num_created =
                        createUnmatchedObjects(frame_instance_matches, frame_instance_images, frame, camera_pose);
                    bg_instance = createBgInstanceImage(frame, object_renders, frame_instance_images);
                    if (num_created)
                    {
                        needs_optimization = true;
                    }
                    mapper_status = MapperStatus::VALID;
                    break;
                }
            }
            default: break;
        }

        auto deleted_object_keys = map_->deleteBadObjects();
        //! TODO: Delete factors and values associated with the key
        //! Stores iterators to the factors associated with keys to be deleted
        std::set<size_t> removed_factor_slots;
        const gtsam::VariableIndex variable_index(pose_graph_);
        for (const auto& object_key : deleted_object_keys)
        {
            if (pose_values_.exists(object_key))
            {
                const auto& slots = variable_index[object_key];
                removed_factor_slots.insert(slots.begin(), slots.end());
                pose_values_.erase(object_key);
            }
        }
        //! TODO: Replace the removed factors with marginalized factor??
        for (size_t slot : removed_factor_slots)
        {
            if (pose_graph_.at(slot))
            {
                pose_graph_.remove(slot);
            }
        }

        if (shouldCreateNewBackground(curr_timestamp_))
        {
            map_->addBackground(createBackground(frame, map_->getCameraPose(curr_timestamp_)));
            spdlog::info("Added new background object into the map");
            needs_optimization = true;
        }

        spdlog::info("Total number of objects in the map: {}", map_->getNumObjects());
        if (needs_optimization)
        {
            gtsam::LevenbergMarquardtOptimizer optimizer = gtsam::LevenbergMarquardtOptimizer(pose_graph_, pose_values_);
            gtsam::Values new_values                     = optimizer.optimize();
            spdlog::info("Graph error before optimize: {}", pose_graph_.error(pose_values_));
            spdlog::info("Graph error after optimize: {}", pose_graph_.error(new_values));
            map_->update(new_values, keyframe_timestamps_);
            pose_values_ = new_values;

            //! Create new background here with updated keyframe_timestamp_ pose
            map_->addBackground(createBackground(frame, map_->getCameraPose(curr_timestamp_)));
            mapper_status = MapperStatus::OPTIMIZED;
        }
        spdlog::info("Mapper took {} ms", Timer::toc(start_time).count());
        return std::make_unique<RendererInput>(curr_timestamp_, mapper_status, bg_instance, object_renders, frame);
    }

    void Mapper::initializeMapAndGraph(const Frame& frame,
                                       const InstanceImages& instance_images,
                                       const Eigen::Matrix4d& camera_pose)
    {
        spdlog::trace("Mapper::initializeMapAndGraph()");
        //! Add to the map
        map_->addCameraPose(camera_pose);

        //! Add prior and value to the graph
        addPriorFactor(curr_timestamp_, camera_pose);
        addCameraValue(curr_timestamp_, camera_pose);

        //! Create background for tracking
        map_->addBackground(createBackground(frame, camera_pose));

        // Add each mask instance in the first image as object in map
        for (const auto& instance_image : instance_images)
        {
            TSDFObject::UniquePtr object = createObject(frame, instance_image, camera_pose);
            if (!object)
            {
                spdlog::debug("Object construction failed!");
                continue;
            }

            auto object_key                    = object->hash(object->id_);
            Eigen::Matrix4d T_object_to_camera = camera_pose.inverse() * object->getPose();

            //! Add between factor and value to the graph
            addObjectCameraBetweenFactor(object_key, curr_timestamp_, T_object_to_camera);
            addObjectValue(object_key, object->getPose());

            //! Add object to the map
            map_->addObject(std::move(object));
        }
    }

    void Mapper::projectInstanceImages(const Timestamp& keyframe_timestamp,
                                       const Frame& frame,
                                       const InstanceImages& instance_images,
                                       InstanceImages& projected_instance_images)
    {
        spdlog::trace("Mapper::projectInstanceImages()");
        auto T_keyframe_to_world             = map_->getCameraPose(keyframe_timestamp);
        auto T_camera_to_world               = map_->getCameraPose(curr_timestamp_);
        Eigen::Matrix4d T_keyframe_to_camera = T_camera_to_world.inverse() * T_keyframe_to_world;
        cuda::TransformCuda T_keyframe_to_camera_cuda;
        T_keyframe_to_camera_cuda.FromEigen(T_keyframe_to_camera);
        cuda::PinholeCameraIntrinsicCuda intrinsic_cuda(frame.intrinsic_);

        projected_instance_images.reserve(instance_images.size());
        for (const auto& instance_image : instance_images)
        {
            spdlog::debug("Projecting instance image with label: {}, score: {}", instance_image.label_, instance_image.score_);
            //! Project the mask from maskframe to current frame
            cuda::ImageCuda<uchar, 1> src_mask;
            src_mask.Upload(instance_image.maskb_);

            cv::Mat depthf;
            cv::rgbd::rescaleDepth(frame.depth_, CV_32FC1, depthf, frame.depth_factor_);
            cuda::ImageCuda<float, 1> depth;
            depth.Upload(depthf);

            // TODO: Use geometric segmentation
            auto proj_mask_cuda =
                cuda::SegmentationCuda::TransformAndProject(src_mask, depth, T_keyframe_to_camera_cuda, intrinsic_cuda);
            cv::Mat proj_mask = proj_mask_cuda.DownloadMat();
            BoundingBox proj_bbox;
            //! Do not use mask if it is too close to border
            if(!transform_project_bbox(instance_image.bbox_, proj_bbox, depthf, frame.intrinsic_, T_keyframe_to_camera))
                continue;
            projected_instance_images.emplace_back(
                proj_mask, proj_bbox, instance_image.label_, instance_image.score_, instance_image.feature_);
        }
    }

    InstanceImage Mapper::createBgInstanceImage(const Frame& frame,
                                                const Renders& object_renders,
                                                const InstanceImages& instance_images) const
    {
        cv::Mat bg_mask = cv::Mat::zeros(frame.height_, frame.width_, CV_8UC1);
        for(const auto& render_pair : object_renders)
        {
            const Render& object_render = render_pair.second;
            cv::Mat gray;
            cv::cvtColor(object_render.color_map_, gray, cv::COLOR_BGR2GRAY);
            cv::Mat mask;
            cv::threshold(gray, mask, 10, 255, cv::THRESH_BINARY);
            cv::bitwise_or(bg_mask, mask, bg_mask);
        }
        for (const auto& instance_image : instance_images)
        {
            cv::bitwise_or(bg_mask, instance_image.maskb_, bg_mask);
        }

        cv::Mat im_floodfill = bg_mask.clone();
        cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
        bg_mask = ~(bg_mask | ~im_floodfill);
        return InstanceImage(bg_mask, BoundingBox({ 0, 0, frame.width_, frame.height_ }), 0, 1);
    }

    std::vector<bool> Mapper::integrateObjects(const Renders& object_renders,
                                               const InstanceImages& frame_instance_images,
                                               const Frame& frame,
                                               const Eigen::Matrix4d& camera_pose)
    {
        //! NOTE: Object renders size is 1 less than frame_instance_images size
        std::vector<bool> frame_instance_matches(frame_instance_images.size(), false);
        spdlog::debug("Frame instance images size: {}, Frame instance matches size: {}",
                      frame_instance_images.size(),
                      frame_instance_matches.size());
        for (const auto& render_pair : object_renders)
        {
            const ObjectId& id       = render_pair.first;
            const Render& render     = render_pair.second;
            const cv::Mat& color_map = render.color_map_;

            auto iter = associateObjects(id, color_map, frame_instance_images, frame_instance_matches);
            //! No match
            if (iter == frame_instance_images.end())
            {
                //! Increment the non-existence count
                map_->incrementNonExistence(id);
            }
            //! Match
            else
            {
                map_->incrementExistence(id);
                map_->integrateObject(id, frame, *iter, camera_pose);
                if (curr_timestamp_ == keyframe_timestamps_.back())
                {
                    Eigen::Matrix4d T_object_to_camera = camera_pose.inverse() * map_->getObjectPose(id);
                    addObjectCameraBetweenFactor(map_->getObjectHash(id), curr_timestamp_, T_object_to_camera);
                }
            }
        }
        for(auto match: frame_instance_matches)
            spdlog::debug("Match: {}", match);
        return frame_instance_matches;
    }

    unsigned int Mapper::createUnmatchedObjects(const std::vector<bool>& instance_matches,
                                                const InstanceImages& instance_images,
                                                const Frame& frame,
                                                const Eigen::Matrix4d& camera_pose)
    {
        unsigned int objects_created = 0;
        for (size_t i = 0; i < instance_matches.size(); i++)
        {
            if (!instance_matches.at(i))
            {
                TSDFObject::UniquePtr object = createObject(frame, instance_images.at(i), camera_pose);
                if (!object)
                {
                    spdlog::debug("Object creation failed!");
                    continue;
                }
                auto object_key                      = object->hash(object->id_);
                auto prev_keyframe_timestamp         = keyframe_timestamps_.back();
                Eigen::Matrix4d T_keyframe_to_world  = map_->getCameraPose(prev_keyframe_timestamp);
                Eigen::Matrix4d T_object_to_keyframe = T_keyframe_to_world.inverse() * object->getPose();

                addObjectCameraBetweenFactor(object_key, keyframe_timestamps_.back(), T_object_to_keyframe);
                addObjectValue(object_key, object->getPose());
                spdlog::info("Added object into the map");
                map_->addObject(std::move(object));
                objects_created++;
            }
            else
            {
                spdlog::debug("Object instance {} matched", i);
            }
        }
        return objects_created;
    }

    TSDFObject::UniquePtr Mapper::createBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose)
    {
        using namespace open3d;
        //! Create default instance image for background
        InstanceImage background_instance(frame.width_, frame.height_);
        ObjectId background_id(0, frame.timestamp_, BoundingBox({ 0, 0, frame.width_ - 1, frame.height_ - 1 }));

        TSDFObject::UniquePtr background =
            std::make_unique<TSDFObject>(background_id, frame, background_instance, camera_pose, BACKGROUND_RESOLUTION);

        background->integrate(frame, background_instance, camera_pose);

        return background;
    }

    bool Mapper::shouldCreateNewBackground(Timestamp timestamp)
    {
        double vis_ratio = map_->getBackgroundVisibilityRatio(timestamp);
        spdlog::debug("Visibility Ratio: {}", vis_ratio);

        if (vis_ratio < 0.2)
            return true;
        return false;
    }

    TSDFObject::UniquePtr Mapper::createObject(const Frame& frame,
                                               const InstanceImage& instance_image,
                                               const Eigen::Matrix4d& camera_pose)
    {
        if (instance_image.score_ < SCORE_THRESHOLD)
        {
            spdlog::debug(
                "Object {} with score {} is below score threshold. Not added", instance_image.label_, instance_image.score_);
            return nullptr;
        }
        // If the masksize is smaller that 50^2 pixels
        if (cv::countNonZero(instance_image.maskb_) < 2500)
        {
            spdlog::debug("Object {} width score {} is too small. Not added", instance_image.label_, instance_image.score_);
            return nullptr;
        }

        Eigen::Vector2i object_center = Eigen::Vector2i((instance_image.bbox_[0] + instance_image.bbox_[2]) / 2,
                                                        (instance_image.bbox_[1] + instance_image.bbox_[3]) / 2);
        spdlog::debug("Object center: {}", object_center);
        if (!(object_center[0] >= InstanceImage::BORDER_WIDTH &&
              object_center[0] < frame.width_ - InstanceImage::BORDER_WIDTH &&
              object_center[1] >= InstanceImage::BORDER_WIDTH &&
              object_center[1] < frame.height_ - InstanceImage::BORDER_WIDTH))
        {
            spdlog::debug("Object near corner of the image, Object center: {}", object_center);
            return nullptr;
        }

        ObjectId object_id(instance_image.label_, frame.timestamp_, instance_image.bbox_);

        TSDFObject::UniquePtr object =
            std::make_unique<TSDFObject>(object_id, frame, instance_image, camera_pose, OBJECT_RESOLUTION);

        object->integrate(frame, instance_image, camera_pose);

        return object;
    }

    InstanceImages::const_iterator Mapper::associateObjects(const ObjectId& id,
                                                            const cv::Mat& object_render_color,
                                                            const InstanceImages& instance_images,
                                                            std::vector<bool>& instance_matches) const
    {
        cv::Mat gray;
        cv::cvtColor(object_render_color, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask;
        cv::threshold(gray, mask, 10, 255, cv::THRESH_BINARY);

        auto iter = instance_images.begin();
        std::vector<std::tuple<double, double, size_t>> quality;
        for (; iter != instance_images.end(); ++iter)
        {
            cv::Mat intersection_mask, union_mask;
            cv::bitwise_and(iter->maskb_, mask, intersection_mask);
            cv::bitwise_or(iter->maskb_, mask, union_mask);

            int union_val        = cv::countNonZero(union_mask);
            int intersection_val = cv::countNonZero(intersection_mask);

            //! Object did not get rendered, nor was there a detection
            double geometric_quality = static_cast<double>(intersection_val) / static_cast<double>(union_val);
            if (union_val == 0)
                geometric_quality = 0.0;
            quality.emplace_back(geometric_quality, map_->computeObjectMatch(id, iter->feature_), iter-instance_images.begin());
        }
        std::sort(quality.begin(), quality.end(), [](decltype(quality)::value_type val1, decltype(quality)::value_type val2) {
            return std::get<1>(val1) < std::get<1>(val2);
        });
        spdlog::debug("Matching objectID: {}", id);
        for(auto qual : quality)
        {
            auto geometric_qual = std::get<0>(qual);
            auto semantic_qual = std::get<1>(qual);
            auto idx = std::get<2>(qual);
            spdlog::debug("{}, {}, {}", geometric_qual, semantic_qual, idx);
            if(!instance_matches.at(idx) && geometric_qual > IOU_OVERLAP_THRESHOLD)
            {
                instance_matches.at(idx) = true;
                return instance_images.begin() + idx;
            }
        }
        return instance_images.end();
    }

inline void Mapper::addPriorFactor(const Timestamp& timestamp, const Eigen::Matrix4d& camera_pose)
{
    spdlog::trace("Mapper::addPriorFactor");
    auto camera_key = gtsam::Symbol('c', timestamp);
    // Prior noise of 0.01 metres (x,y,z) and 0.01 rad in (alpha, beta, gamma)
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.01));
    pose_graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(camera_key, gtsam::Pose3(camera_pose), prior_noise);
}

inline void Mapper::addCameraCameraBetweenFactor(const Timestamp& time_source_camera,
                                                 const Timestamp& time_target_camera,
                                                 const Eigen::Matrix4d& T_source_camera_to_target_camera)
{
    spdlog::trace("Mapper::addCameraCameraBetweenFactor");
    auto source_camera_key = gtsam::Symbol('c', time_source_camera);
    auto target_camera_key = gtsam::Symbol('c', time_target_camera);

    //! TODO: Obtain noise from ICP
    auto between_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-3));
    pose_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        target_camera_key, source_camera_key, gtsam::Pose3(T_source_camera_to_target_camera), between_noise);
}

inline void Mapper::addObjectCameraBetweenFactor(const gtsam::Key& object_key,
                                                 const Timestamp& camera_timestamp,
                                                 const Eigen::Matrix4d& T_object_to_camera)
{
    spdlog::trace("Mapper::addObjectCameraBetweenFactor");
    auto camera_key = gtsam::Symbol('c', camera_timestamp);

    auto between_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-3));
    pose_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        camera_key, object_key, gtsam::Pose3(T_object_to_camera), between_noise);
}

inline void Mapper::addCameraValue(const Timestamp& timestamp, const Eigen::Matrix4d& camera_pose)
{
    //! TODO: Error checking
    spdlog::trace("Mapper::addCameraValue");
    auto camera_key = gtsam::Symbol('c', timestamp);
    pose_values_.insert(camera_key, gtsam::Pose3(camera_pose));
}

inline void Mapper::addObjectValue(const gtsam::Key& object_key, const Eigen::Matrix4d& object_pose)
{
    spdlog::trace("Mapper::addObjectValue");
    pose_values_.insert(object_key, gtsam::Pose3(object_pose));
}
}  // namespace oslam
