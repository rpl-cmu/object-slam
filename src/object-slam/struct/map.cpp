/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "object-slam/utils/utils.h"
#include "object-slam/struct/instance_image.h"

namespace oslam
{
    void Map::addBackground(TSDFObject::UniquePtr bg)
    {
        std::scoped_lock<std::mutex> lock_add_background(mutex_);
        if (background_volume_)
        {
            background_volume_.reset(nullptr);  //! Delete the underlying background volume
            spdlog::debug("Deleted previously existing background volume");
        }
        background_volume_ = std::move(bg);
        spdlog::debug("Added new background volume");
    }

    void Map::integrateBackground(const Frame& frame,
                                  const InstanceImage& instance_image,
                                  const Eigen::Matrix4d& camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integrate_bg(mutex_);
        background_volume_->integrate(frame, instance_image, camera_pose);
    }

    Render Map::renderBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose)
    {
        std::scoped_lock<std::mutex> lock_render_background(mutex_);
        cuda::ImageCuda<float, 3> vertex;
        cuda::ImageCuda<float, 3> normal;
        cuda::ImageCuda<uchar, 3> color;

        vertex.Create(frame.width_, frame.height_);
        normal.Create(frame.width_, frame.height_);
        color.Create(frame.width_, frame.height_);

        background_volume_->raycast(vertex, normal, color, camera_pose);

        cv::Mat color_map  = color.DownloadMat();
        cv::Mat vertex_map = vertex.DownloadMat();
        cv::Mat normal_map = normal.DownloadMat();

        return Render(color_map, vertex_map, normal_map);
    }

    double Map::getBackgroundVisibilityRatio(const Timestamp& timestamp) const
    {
        std::scoped_lock<std::mutex> lock_vis_ratio(mutex_);
        return background_volume_->getVisibilityRatio(timestamp);
    }

    bool Map::addObject(TSDFObject::UniquePtr object)
    {
        std::scoped_lock<std::mutex> lock_add_object(mutex_);
        ObjectId curr_object_id = object->id_;
        if (curr_object_id.label == 0)
        {
            spdlog::warn("Trying to add background object in map hashtable, returning");
            return false;
        }
        auto success = id_to_object_.insert(std::make_pair(curr_object_id, std::move(object)));
        spdlog::debug("Added new entry: {} to the map, new size: {}", curr_object_id, id_to_object_.size());
        return success.second;
    }

    void Map::integrateObject(const ObjectId &id, const Frame &frame, const InstanceImage &instance_image, const Eigen::Matrix4d &camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integrate_object(mutex_);
        TSDFObject::Ptr& object = id_to_object_.at(id);
        object->integrate(frame, instance_image, camera_pose);
    }

    Renders Map::renderObjects(const Frame& frame, const Eigen::Matrix4d& camera_pose)
    {
        std::scoped_lock<std::mutex> lock_render_objects(mutex_);
        Renders object_renders;
        object_renders.reserve(id_to_object_.size());
        for (auto& object_pair : id_to_object_)
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

            object_renders.emplace_back(id, Render(color_map, vertex_map, normal_map));
        }
        return object_renders;
    }

    std::uint64_t Map::getObjectHash(const ObjectId& id) const
    {
        std::scoped_lock<std::mutex> lock_get_object_hash(mutex_);
        const TSDFObject::Ptr& object = id_to_object_.at(id);
        return object->hash(id);
    }
    Eigen::Matrix4d Map::getObjectPose(const ObjectId& id) const
    {
        std::scoped_lock<std::mutex> lock_get_object_pose(mutex_);
        const TSDFObject::Ptr& object = id_to_object_.at(id);
        return object->getPose();
    }

    void Map::deleteBadObjects()
    {
        std::scoped_lock<std::mutex> lock_delete_objects(mutex_);
        // Evaluate remove objects with very low existence probability
        std::vector<ObjectId> to_delete_objects;
        for (const auto& object_pair : id_to_object_)
        {
            const ObjectId id           = object_pair.first;
            TSDFObject::ConstPtr object = object_pair.second;

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
            removeObject(object_id);
        }
    }

    bool Map::removeObject(const ObjectId& id)
    {
        auto it = id_to_object_.find(id);
        if(it == id_to_object_.end())
        {
            spdlog::error("Fatal: Object to delete does not exist in the map");
            return false;
        }
        id_to_object_.erase(it);
        return true;
    }
    void Map::incrementExistence(const ObjectId& id)
    {
        std::scoped_lock<std::mutex> lock_incr_existence(mutex_);
        auto object = id_to_object_.at(id);
        object->existence_++;
        spdlog::debug("Object existence: {}", object->existence_);
    }
    void Map::incrementNonExistence(const ObjectId& id)
    {
        std::scoped_lock<std::mutex> lock_incr_non_existence(mutex_);
        auto object = id_to_object_.at(id);
        object->non_existence_++;
        spdlog::debug("Object non-existence: {}", object->non_existence_);
    }

    void Map::addCameraPose(const Eigen::Matrix4d& camera_pose)
    {
        std::scoped_lock<std::mutex> lock_add_camera_pose(mutex_);
        camera_trajectory_.push_back(camera_pose);
    }

    Eigen::Matrix4d Map::getCameraPose(const Timestamp& camera_timestamp)
    {
        std::scoped_lock<std::mutex> lock_get_camera_pose(mutex_);
        if (camera_timestamp > camera_trajectory_.size())
        {
            spdlog::error("Requesting camera pose with timestamp: {}, but camera_trajectory size: {}",
                          camera_timestamp,
                          camera_trajectory_.size());
            return Eigen::Matrix4d::Identity();
        }

        return camera_trajectory_.at(camera_timestamp - 1);
    }

    PoseTrajectory Map::getCameraTrajectory() const
    {
        std::scoped_lock<std::mutex> lock_get_cam_trajectory(mutex_);
        return camera_trajectory_;
    }

    void Map::update(const gtsam::Values& values, const std::vector<Timestamp>& keyframe_timestamps)
    {
        std::scoped_lock<std::mutex> lock_update_map(mutex_);

        for (auto& object_pair : id_to_object_)
        {
            const ObjectId& id      = object_pair.first;
            TSDFObject::Ptr& object = object_pair.second;

            if (values.exists(object->hash(id)))
            {
                gtsam::Pose3 updatedPose = values.at<gtsam::Pose3>(object->hash(id));
                object->setPose(updatedPose.matrix());
            }
        }
        for (const auto& keyframe_timestamp : keyframe_timestamps)
        {
            auto camera_key = gtsam::Symbol('c', keyframe_timestamp);
            if (values.exists(camera_key))
            {
                gtsam::Pose3 updatedPose                                 = values.at<gtsam::Pose3>(camera_key);
                camera_trajectory_.at(keyframe_timestamp - 1) = updatedPose.matrix();
            }
        }

    }

}  // namespace oslam
