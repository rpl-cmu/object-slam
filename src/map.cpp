/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Open3D/Camera/PinholeCameraIntrinsic.h>
/* #include <gtsam/geometry/Pose3.h> */
/* #include <gtsam/inference/Symbol.h> */
/* #include <gtsam/slam/BetweenFactor.h> */
/* #include <gtsam/slam/PriorFactor.h> */
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <vector>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    std::pair<ObjectId, TSDFObject::Ptr> GlobalMap::create_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose)
    {
        using namespace open3d;

        //! Create default instance image for background
        InstanceImage background_instance(r_frame.m_width, r_frame.m_height);
        ObjectId background_id(0, r_frame.m_timestamp, BoundingBox({ 0, 0, r_frame.m_width - 1, r_frame.m_height - 1 }));
        TSDFObject::Ptr p_background = std::make_shared<TSDFObject>(r_frame, background_instance, r_camera_pose, 256);
        p_background->integrate(r_frame, background_instance, r_camera_pose);
        auto background_pair = std::make_pair(background_id, p_background);
        auto success_pair = m_id_to_object.insert(background_pair);

        mp_active_background = p_background;
        if (success_pair.second == true)
        {
            spdlog::debug("Added background");
        }
        else
        {
            spdlog::error("Unable to add background into global map at frame: {}", r_frame.m_timestamp);
        }

        return background_pair;
    }

    std::pair<ObjectId, TSDFObject::Ptr> GlobalMap::create_object(const Frame &r_frame, const InstanceImage &r_instance_image,
                                      const Eigen::Matrix4d &r_camera_pose)
    {
        //! Don't allow other objects to be added to vector
        if (r_instance_image.m_score < SCORE_THRESHOLD)
        {
            spdlog::warn("Object {} with score {} is below score threshold. Not added", r_instance_image.m_label,
                         r_instance_image.m_score);
            return std::make_pair(ObjectId(), nullptr);
        }
        //! If the masksize is smaller that 50^2 pixels
        if (cv::countNonZero(r_instance_image.m_maskb) < 2500)
        {
            spdlog::warn("Object {} width score {} is too small. Not added", r_instance_image.m_label,
                         r_instance_image.m_score);
            return std::make_pair<ObjectId, TSDFObject::Ptr>(ObjectId(), nullptr);
        }

        ObjectId object_id(r_instance_image.m_label, r_frame.m_timestamp, r_instance_image.m_bbox);
        if (m_id_to_object.find(object_id) != m_id_to_object.end())
        {
            spdlog::error("Object {} with score {} already exists in map", r_instance_image.m_label,
                          r_instance_image.m_score);
            return std::make_pair<ObjectId, TSDFObject::Ptr>(ObjectId(), nullptr);
        }
        Eigen::Vector2i object_center = Eigen::Vector2i((r_instance_image.m_bbox[0] + r_instance_image.m_bbox[2])/2,
                                                        (r_instance_image.m_bbox[1] + r_instance_image.m_bbox[3])/2);
        if (!(object_center[0] >= InstanceImage::BORDER_WIDTH &&
            object_center[0] < r_frame.m_width - InstanceImage::BORDER_WIDTH &&
            object_center[1] >= InstanceImage::BORDER_WIDTH &&
            object_center[1] < r_frame.m_height - InstanceImage::BORDER_WIDTH))
        {
            spdlog::debug("Object near corner of the image, Object center: {}", object_center);
            return std::make_pair<ObjectId, TSDFObject::Ptr>(ObjectId(), nullptr);
        }
        TSDFObject::Ptr p_object = std::make_shared<TSDFObject>(r_frame, r_instance_image, r_camera_pose, 128);
        p_object->integrate(r_frame, r_instance_image, r_camera_pose);
        auto object_pair = std::make_pair(object_id, std::move(p_object));
        auto success_pair = m_id_to_object.insert(object_pair);
        if (success_pair.second == true)
        {
            spdlog::info("Created and added new object: {}, {}, {}", r_instance_image.m_label, r_instance_image.m_score, object_center);
        }
        else
        {
            spdlog::error("Unable to add object: {} into global map at frame: {}", r_instance_image.m_label,
                          r_frame.m_timestamp);
        }
        return object_pair;
    }

    TSDFObject::ConstPtr GlobalMap::get_object(const ObjectId &r_id) const
    {
        IdToObjectMap::const_iterator it = m_id_to_object.find(r_id);
        if (it == m_id_to_object.end())
        {
            spdlog::error("Fatal: could not find object in the map");
            return nullptr;
        }

        return it->second;
    }

    bool GlobalMap::integrate_background(const Frame& r_frame, const Eigen::Matrix4d& r_camera_pose)
    {
        InstanceImage background_instance(r_frame.m_width, r_frame.m_height);
        mp_active_background->integrate(r_frame, background_instance, r_camera_pose);
        spdlog::debug("Integrated background");
    }

    bool GlobalMap::integrate_object(const Frame &r_frame, const InstanceImage &r_instance_image,
                                     const Eigen::Matrix4d &r_camera_pose)
    {
        if (r_instance_image.m_score < SCORE_THRESHOLD)
        {
            spdlog::warn("Object label {} with score {} is below score threshold. Not added", r_instance_image.m_label, r_instance_image.m_score);
            return false;
        }

        bool matched = false;
        //! Project the input binary mask to the current frame
        //! Try to match the src_proj_mask with raycasted mask of existing objects in map
        for (auto i = m_id_to_object.begin(); i != m_id_to_object.end(); ++i)
        {
            ObjectId matching_label       = i->first;
            TSDFObject::Ptr p_matching_object = i->second;

            //! Do not associate if the map object is background!!
            if(p_matching_object->is_background())
                continue;

            Eigen::Vector4d object_center              = p_matching_object->get_pose().block<4, 1>(0, 3);
            Eigen::Vector4d object_center_camera_frame = r_camera_pose * object_center;
            auto proj_object_center = project_point(object_center_camera_frame.head(3), r_frame.m_intrinsic);

            //! 2. Check if the map object center is in the camera frame
            /* if (proj_object_center[0] >= InstanceImage::BORDER_WIDTH && */
            /*     proj_object_center[0] < r_frame.m_width - InstanceImage::BORDER_WIDTH && */
            /*     proj_object_center[1] >= InstanceImage::BORDER_WIDTH && */
            /*     proj_object_center[1] < r_frame.m_height - InstanceImage::BORDER_WIDTH) */
            {
                //! 3. Raycast the map object to obtain the raycasted mask
                cuda::ImageCuda<uchar, 3> color;
                cuda::ImageCuda<float, 3> vertex, normal;
                vertex.Create(r_frame.m_width, r_frame.m_height, 0);
                normal.Create(r_frame.m_width, r_frame.m_height, 0);
                color.Create(r_frame.m_width, r_frame.m_height, 0);

                p_matching_object->raycast(vertex, normal, color, r_camera_pose);
                auto color_image = color.DownloadMat();
                /* cv::imshow(fmt::format("Raycasted object label: {}", p_matching_object->get_label()), color_image); */

                cv::Mat mat_gray, mat_mask;
                cv::cvtColor(color_image, mat_gray, cv::COLOR_BGR2GRAY);
                cv::threshold(mat_gray, mat_mask, 10, 255, cv::THRESH_BINARY);
                int mask_area = cv::countNonZero(mat_mask);

                //! 5. Compute the intersection of projected input mask and raycasted mask
                cv::Mat intersection_mask, union_mask;
                cv::bitwise_and(r_instance_image.m_maskb, mat_mask, intersection_mask);
                cv::bitwise_or(r_instance_image.m_maskb, mat_mask, union_mask);

                /* cv::imshow("Intersection mask", intersection_mask); */
                auto quality = static_cast<float>(cv::countNonZero(intersection_mask)) /
                               static_cast<float>(cv::countNonZero(union_mask));
                spdlog::debug("Quality of the association: {}, Source label: {}, Target label: {}", quality,
                              r_instance_image.m_label, matching_label);

                //! 5. Finally integrate the object
                if (quality > 0.2f)
                {
                    p_matching_object->integrate(r_frame, r_instance_image, r_camera_pose);
                    matched = true;
                }
                vertex.Release();
                normal.Release();
                color.Release();
            }
        }
        return matched;
    }

    void GlobalMap::raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                                       open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose)
    {
        //! Always raycast with the last background entry in the multimap
        //! TODO: Raycast layered background with objects for better vertices and normals
        mp_active_background->raycast(vertex, normal, color, r_camera_pose);
        spdlog::debug("Raycasted background");
    }

}  // namespace oslam
