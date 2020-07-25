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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/rgbd/depth.hpp>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    Mapper::Mapper(Map::Ptr map,
                   TrackerOutputQueue* tracker_output_queue,
                   TransportOutputQueue* transport_output_queue,
                   OutputQueue* output_queue)
        : MISO(output_queue, "Mapper"),
          map_(map),
          active_bg_id_(ObjectId()),
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
        const bool is_current_maskframe = mapper_payload->frame_.is_maskframe_;

        Mapper::InputUniquePtr mapper_input;
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            mapper_input = std::make_unique<MapperInput>(curr_timestamp_,
                                                         prev_maskframe_timestamp_,
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

            prev_transport_output_    = std::move(transport_output);
            prev_maskframe_timestamp_ = curr_timestamp_;
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
        const Frame& frame                          = mapper_payload.frame_;
        const InstanceImages& instance_images       = mapper_payload.instance_images_;
        const Eigen::Matrix4d& relative_camera_pose = mapper_payload.relative_camera_pose_;
        MapperStatus mapper_status                  = MapperStatus::INVALID;

        if (curr_timestamp_ == 1)
        {
            // Instantiate objects and background in map
            if (mapper_payload.tracker_status_ == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d camera_pose = relative_camera_pose;
                T_camera_to_world_.emplace_back(camera_pose);

                // Add camera pose and factor to the graph
                TSDFObject::Ptr bg = createBackground(frame, camera_pose);
                active_bg_id_      = bg->id_;
                auto bg_key        = bg->hash(bg->id_);

                // Prior noise of 0.01 metres (x,y,z) and 0.01 rad in (alpha, beta, gamma)
                auto prior_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.01));
                spdlog::debug("Background object pose:\n {}", bg->getPose());
                object_pose_graph_.emplace_shared<PriorFactor<Pose3>>(bg_key, Pose3(bg->getPose()), prior_noise);
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
                    auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));
                    spdlog::debug("Current object pose:\n {}", object->getPose());
                    object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                        bg_key, object_key, Pose3(object->getPose()), between_noise);
                    map_->addObject(object);
                }
                mapper_status = MapperStatus::VALID;
            }
        }
        else
        {
            if (mapper_payload.tracker_status_ == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d camera_pose = T_camera_to_world_.back() * relative_camera_pose;
                T_camera_to_world_.emplace_back(camera_pose);

                TSDFObject::Ptr active_bg = map_->getBackground();
                active_bg->integrate(frame, InstanceImage(frame.width_, frame.height_), camera_pose);

                std::vector<std::pair<ObjectId, cv::Mat>> object_raycasts;
                raycastMapObjects(object_raycasts, frame, camera_pose);

                //! Project the masks to current frame
                auto T_maskcamera_to_world             = T_camera_to_world_.at(prev_maskframe_timestamp_ - 1);
                auto T_camera_to_world                 = T_camera_to_world_.back();
                Eigen::Matrix4d T_maskcamera_to_camera = T_camera_to_world.inverse() * T_maskcamera_to_world;
                unsigned int i                         = 0;

                cuda::TransformCuda T_maskcamera_to_camera_cuda;
                T_maskcamera_to_camera_cuda.FromEigen(T_maskcamera_to_camera);
                cuda::PinholeCameraIntrinsicCuda intrinsic_cuda(frame.intrinsic_);

                for (auto& instance_image : instance_images)
                {
                    //! Skip for background object
                    if (instance_image.label_ == 0)
                        continue;

                    spdlog::debug("Processing {}th instance image in payload: {}", i, curr_timestamp_);
                    //! Project the mask from maskframe to current frame
                    cuda::ImageCuda<uchar, 1> src_mask;
                    src_mask.Upload(instance_image.maskb_);

                    cv::Mat depthf;
                    cv::rgbd::rescaleDepth(frame.depth_, CV_32F, depthf);
                    cuda::ImageCuda<float, 1> depth;
                    depth.Upload(depthf);

                    auto proj_mask_cuda = cuda::SegmentationCuda::TransformAndProject(
                        src_mask, depth, T_maskcamera_to_camera_cuda, intrinsic_cuda);
                    cv::Mat proj_mask = proj_mask_cuda.DownloadMat();

                    BoundingBox proj_bbox =
                        transform_project_bbox(instance_image.bbox_, depthf, frame.intrinsic_, T_maskcamera_to_camera);

#ifdef OSLAM_DEBUG_VIS
                    cv::imshow("Input instance image mask", instance_image.maskb_);
                    cv::imshow("Source projected mask", proj_mask);
#endif
                    InstanceImage proj_instance_image(proj_mask, proj_bbox, instance_image.label_, instance_image.score_);

                    bool matched = false;
                    ObjectId matched_id;
                    std::tie(matched, matched_id) = associateObjects(proj_instance_image, object_raycasts);

                    if (matched)
                    {
                        //! Integrate with object ID matched with
                        TSDFObject::Ptr matched_object = map_->getObject(matched_id);
                        matched_object->integrate(frame, proj_instance_image, T_camera_to_world_.back());
                    }
                    else
                    {
                        //! Create new object instance
                        auto object = createObject(frame, proj_instance_image, camera_pose);
                        if (!object)
                        {
                            spdlog::debug("Object creation failed!");
                            continue;
                        }

                        auto object_key = object->hash(object->id_);
                        auto bg_key     = active_bg->hash(active_bg->id_);

                        auto between_noise = noiseModel::Diagonal::Variances(Vector6::Constant(1e-3));
                        object_pose_graph_.emplace_shared<BetweenFactor<Pose3>>(
                            bg_key, object_key, Pose3(object->getPose()), between_noise);

#ifdef OSLAM_DEBUG_VIS
                        object_pose_graph_.print("Factor Graph:\n");
#endif
                    }
                    i++;
                }
                mapper_status = MapperStatus::VALID;
            }
            // TODO: Delete objects
        }
        return std::make_unique<RendererInput>(curr_timestamp_, mapper_status, frame, T_camera_to_world_.back());
    }

    TSDFObject::Ptr Mapper::createBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose)
    {
        using namespace open3d;
        //! Create default instance image for background
        InstanceImage background_instance(frame.width_, frame.height_);
        ObjectId background_id(0, frame.timestamp_, BoundingBox({ 0, 0, frame.width_ - 1, frame.height_ - 1 }));
        TSDFObject::Ptr background =
            std::make_shared<TSDFObject>(background_id, frame, background_instance, camera_pose, 256);

        background->integrate(frame, background_instance, camera_pose);
        return background;
    }

    TSDFObject::Ptr Mapper::createObject(const Frame& frame,
                                         const InstanceImage& instance_image,
                                         const Eigen::Matrix4d& camera_pose)
    {
        // Don't allow other objects to be added to vector
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

        if (!(object_center[0] >= InstanceImage::BORDER_WIDTH &&
              object_center[0] < frame.width_ - InstanceImage::BORDER_WIDTH &&
              object_center[1] >= InstanceImage::BORDER_WIDTH &&
              object_center[1] < frame.height_ - InstanceImage::BORDER_WIDTH))
        {
            spdlog::debug("Object near corner of the image, Object center: {}", object_center);
            return nullptr;
        }

        ObjectId object_id(instance_image.label_, frame.timestamp_, instance_image.bbox_);
        TSDFObject::Ptr object = std::make_shared<TSDFObject>(object_id, frame, instance_image, camera_pose, 128);
        object->integrate(frame, instance_image, camera_pose);
        return object;
    }

    void Mapper::raycastMapObjects(std::vector<std::pair<ObjectId, cv::Mat>>& object_raycasts,
                                   const Frame& frame,
                                   const Eigen::Matrix4d& camera_pose)
    {
        using namespace open3d;
        for (const auto& object_pair : map_->id_to_object_)
        {
            const ObjectId& id     = object_pair.first;
            TSDFObject::Ptr object = object_pair.second;

            if (object->isBackground())
                continue;

            cuda::ImageCuda<float, 3> vertex, normal;
            cuda::ImageCuda<uchar, 3> color;

            vertex.Create(frame.width_, frame.height_);
            normal.Create(frame.width_, frame.height_);
            color.Create(frame.width_, frame.height_);

            object->raycast(vertex, normal, color, camera_pose);

            cv::Mat mat_raycast = color.DownloadMat();
            object_raycasts.emplace_back(id, mat_raycast);
        }
    }

    std::pair<bool, ObjectId> Mapper::associateObjects(const InstanceImage& instance_image,
                                                       const std::vector<std::pair<ObjectId, cv::Mat>>& object_raycasts)
    {
        if (instance_image.score_ < SCORE_THRESHOLD)
        {
            spdlog::warn("Object label {} with score {} is below score threshold. Not added",
                         instance_image.label_,
                         instance_image.score_);
            return std::make_pair(false, ObjectId());
        }
        for (const auto& pair : object_raycasts)
        {
            ObjectId id                   = pair.first;
            const cv::Mat& object_raycast = pair.second;

            cv::Mat gray, mask;
            cv::cvtColor(object_raycast, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, mask, 10, 255, cv::THRESH_BINARY);

            cv::Mat intersection_mask, union_mask;
            cv::bitwise_and(instance_image.maskb_, mask, intersection_mask);
            cv::bitwise_or(instance_image.maskb_, mask, union_mask);

            auto quality =
                static_cast<float>(cv::countNonZero(intersection_mask)) / static_cast<float>(cv::countNonZero(union_mask));
            spdlog::debug("Quality of the association: {}, Source label: {}, Target label: {}",
                          quality,
                          instance_image.label_,
                          id.label);

            if (quality > 0.2f)
                return std::make_pair(true, id);
        }
        return std::make_pair(false, ObjectId());
    }

}  // namespace oslam
