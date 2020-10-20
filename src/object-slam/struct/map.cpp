/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <Cuda/Open3DCuda.h>
#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <unordered_map>

#include "object-slam/struct/instance_image.h"
#include "object-slam/utils/utils.h"

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

    void Map::integrateObject(const ObjectId& id,
                              const Frame& frame,
                              const InstanceImage& instance_image,
                              const Eigen::Matrix4d& camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integrate_object(mutex_);
        TSDFObject::Ptr& object = id_to_object_.at(id);
        assert(isObjectInFrustum(object, camera_pose));
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

            if (isObjectInFrustum(object, camera_pose))
            {

                if(object->volume_.device_ == nullptr)
                {
                    spdlog::debug("Uploading object to GPU");
                    object->uploadVolumeToGPU();
                }

                //! TODO: Render the objects
                spdlog::info("Current object ID: {} is in frustum", id);
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

                cv::imshow("Object render", color_map);

                object_renders.emplace_back(id, Render(color_map, vertex_map, normal_map));
            }
            else
            {
                //! Download the object onto CPU memory
                spdlog::info("Current object ID: {} is not in frustum", id);
                object->downloadVolumeToCPU();
            }

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

    void Map::getObjectBoundingBox(const ObjectId& id, Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const
    {
        const TSDFObject::Ptr& object = id_to_object_.at(id);
        min_pt                        = object->getMinBound();
        max_pt                        = object->getMaxBound();
    }

    ObjectBoundingBoxes Map::getAllObjectBoundingBoxes() const
    {
        std::scoped_lock<std::mutex> lock_get_bbox(mutex_);
        ObjectBoundingBoxes bboxes;
        for (const auto& object_pair : id_to_object_)
        {
            const ObjectId& id = object_pair.first;
            Eigen::Vector3d min_pt;
            Eigen::Vector3d max_pt;
            getObjectBoundingBox(id, min_pt, max_pt);
            bboxes.emplace(id, std::make_pair(min_pt, max_pt));
        }
        return bboxes;
    }

    IdToObjectMesh Map::meshAllObjects() const
    {
        using namespace open3d::cuda;
        std::scoped_lock<std::mutex> lock_get_mesh(mutex_);
        IdToObjectMesh object_meshes;
        for (const auto& object_pair : id_to_object_)
        {
            const ObjectId& id     = object_pair.first;
            TSDFObject::Ptr object = object_pair.second;
            spdlog::info("Meshing object: {}", id);
            ScalableMeshVolumeCuda object_mesher(
                VertexType::VertexWithColor, 16, object->volume_.active_subvolume_entry_array_.size(), 5000, 10000);

            object_mesher.MarchingCubes(object->volume_);
            auto mesh = object_mesher.mesh().Download();

            object_meshes.insert(std::pair(id, mesh));
        }
        return object_meshes;
    }

    std::vector<std::uint64_t> Map::deleteBadObjects()
    {
        std::scoped_lock<std::mutex> lock_delete_objects(mutex_);
        // Evaluate remove objects with very low existence probability
        std::vector<ObjectId> to_delete_objects;
        std::vector<std::uint64_t> to_delete_object_keys;
        for (const auto& object_pair : id_to_object_)
        {
            const ObjectId id           = object_pair.first;
            TSDFObject::ConstPtr object = object_pair.second;

            double existence_expect = object->getExistExpectation();
            spdlog::debug("{} -> Existence expectation: {}", id, existence_expect);

            if (existence_expect <= 0.2)
            {
                to_delete_objects.push_back(id);
                to_delete_object_keys.push_back(object->hash(id));
            }
        }
        for (const auto& object_id : to_delete_objects)
        {
            spdlog::info("Removing object {}", object_id);
            removeObject(object_id);
        }

        return to_delete_object_keys;
    }

    bool Map::removeObject(const ObjectId& id)
    {
        auto it = id_to_object_.find(id);
        if (it == id_to_object_.end())
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
    bool Map::isObjectInFrustum(const TSDFObject::Ptr& object, const Eigen::Matrix4d& camera_pose)
    {
        auto point_planes = getCameraFrustumPlanes(camera_pose);

        Eigen::Vector3d min_point, max_point;
        min_point = object->getMinBound();
        max_point = object->getMaxBound();

        bool result = true;
        int out = 0;
        int in = 0;
        auto get_vertex = [&min_point, &max_point](int index) -> Eigen::Vector3d {
            Eigen::Vector3d vertex = min_point;

            double x = max_point(0) - min_point(0);
            double y = max_point(1) - min_point(1);
            double z = max_point(2) - min_point(2);

            if(index >= 8)
                spdlog::error("Only 8 vertices");

            std::vector<Eigen::Vector3d> box_vertices;
            box_vertices.emplace_back(min_point + Eigen::Vector3d(0, 0, 0));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(0, 0, z));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(0, y, 0));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(0, y, z));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(x, 0, 0));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(x, 0, z));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(x, y, 0));
            box_vertices.emplace_back(min_point + Eigen::Vector3d(x, y, z));

            return box_vertices.at(index);
        };

        for (const auto& point_plane : point_planes)
        {
            const Plane3d& plane                = point_plane.first;
            const Eigen::Vector3d& plane_center = point_plane.second;
            out = 0; in = 0;
            for(int k = 0; k < 8 && (in ==0 || out == 0); k++)
            {
                if(plane.signedDistance(get_vertex(k)) < 0)
                    out++;
                else
                    in++;
            }
        }
        if(in == 0)
            result = false;
        return result;
    }

    PointPlanes Map::getCameraFrustumPlanes(const Eigen::Matrix4d& camera_pose) const
    {
        //! Z axis is the look ahead vector for the camera
        const Eigen::Vector3d& camera_x        = camera_pose.block<3, 1>(0, 0);
        const Eigen::Vector3d& camera_y        = camera_pose.block<3, 1>(0, 1);
        const Eigen::Vector3d& camera_z        = camera_pose.block<3, 1>(0, 2);
        const Eigen::Vector3d& camera_position = camera_pose.block<3, 1>(0, 3);
        /* spdlog::info("Camera position: \n{}", camera_position); */
        /* spdlog::info("Camera look ahead vector \n{}", camera_z); */

        auto intrinsic = background_volume_->intrinsic_;
        Eigen::Vector2i top_center(intrinsic.width_ / 2, 0), bottom_center(intrinsic.width_ / 2, intrinsic.height_);
        Eigen::Vector2i left_center(0, intrinsic.height_ / 2), right_center(intrinsic.width_, intrinsic.height_ / 2);

        Eigen::Vector3d near_plane_center = camera_position + camera_z * MIN_DEPTH;
        Eigen::Vector3d far_plane_center  = camera_position + camera_z * MAX_DEPTH;

        Eigen::Vector3d near_top_center    = inverse_project_point(top_center, intrinsic, MIN_DEPTH);
        Eigen::Vector3d near_bottom_center = inverse_project_point(bottom_center, intrinsic, MIN_DEPTH);
        Eigen::Vector3d near_left_center   = inverse_project_point(left_center, intrinsic, MIN_DEPTH);
        Eigen::Vector3d near_right_center  = inverse_project_point(right_center, intrinsic, MIN_DEPTH);

        near_top_center    = Eigen::Affine3d(camera_pose) * near_top_center;
        near_bottom_center = Eigen::Affine3d(camera_pose) * near_bottom_center;
        near_left_center   = Eigen::Affine3d(camera_pose) * near_left_center;
        near_right_center  = Eigen::Affine3d(camera_pose) * near_right_center;

        PointPlanes point_planes;
        //! Near plane
        Plane3d near_plane = Plane3d(camera_z, near_plane_center);
        near_plane.normalize();
        point_planes.emplace_back(near_plane, near_plane_center);

        //! Far plane
        Plane3d far_plane = Plane3d(-camera_z, far_plane_center);
        far_plane.normalize();
        point_planes.emplace_back(far_plane, far_plane_center);

        //! Top plane
        Eigen::Vector3d auxiliary, normal;
        auxiliary = near_top_center - camera_position;
        auxiliary.normalize();
        normal = auxiliary.cross(camera_x);
        Plane3d near_top_plane = Plane3d(normal, near_top_center);
        near_top_plane.normalize();
        point_planes.emplace_back(near_top_plane, near_top_center);

        //! Bottom plane
        auxiliary = near_bottom_center - camera_position;
        auxiliary.normalize();
        normal = camera_x.cross(auxiliary);
        Plane3d near_bottom_plane = Plane3d(normal, near_bottom_center);
        near_bottom_plane.normalize();
        point_planes.emplace_back(near_bottom_plane, near_bottom_center);

        //! Left plane
        auxiliary = near_left_center - camera_position;
        auxiliary.normalize();
        normal = camera_y.cross(auxiliary);
        Plane3d near_left_plane = Plane3d(normal, near_left_center);
        near_left_plane.normalize();
        point_planes.emplace_back(near_left_plane, near_left_center);

        //! Right plane
        auxiliary = near_right_center - camera_position;
        auxiliary.normalize();
        normal = auxiliary.cross(camera_y);
        Plane3d near_right_plane = Plane3d(normal, near_right_center);
        near_right_plane.normalize();
        point_planes.emplace_back(near_right_plane, near_right_center);

        return point_planes;
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

            gtsam::Pose3 updatedPose = values.at<gtsam::Pose3>(object->hash(id));
            object->setPose(updatedPose.matrix());
        }
        for (const auto& keyframe_timestamp : keyframe_timestamps)
        {
            auto camera_key = gtsam::Symbol('c', keyframe_timestamp);
            gtsam::Pose3 updatedPose                      = values.at<gtsam::Pose3>(camera_key);
            camera_trajectory_.at(keyframe_timestamp - 1) = updatedPose.matrix();
        }
    }

    double Map::computeObjectMatch(const ObjectId& id, const Feature& feature) const
    {
        std::scoped_lock<std::mutex> lock_integrate_object(mutex_);
        const TSDFObject::Ptr& object = id_to_object_.at(id);
        const Feature& object_feature = object->instance_image_.feature_;

        auto difference = object_feature - feature;
        return cv::norm(difference, cv::NORM_L1);
    }

}  // namespace oslam
