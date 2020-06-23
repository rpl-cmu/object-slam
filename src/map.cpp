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
#include <Cuda/OSLAMUtils/SegmentationCuda.h>
#include <Open3D/Camera/PinholeCameraIntrinsic.h>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    void GlobalMap::add_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose)
    {
        using namespace open3d;
        //! Don't allow other objects to be added to vector
        InstanceImage background_instance(r_frame.m_width, r_frame.m_height);
        TSDFObject::UniquePtr p_background = std::make_unique<TSDFObject>(r_frame, background_instance, r_camera_pose, 256);
        p_background->integrate(r_frame, background_instance, r_camera_pose);
        m_label_to_objects.insert(std::make_pair(0, std::move(p_background)));
        spdlog::debug("Added background");
    }

    void GlobalMap::add_object(const Frame &r_frame, const InstanceImage &r_instance_image,
                               const Eigen::Matrix4d &r_camera_pose)
    {
        //! Don't allow other objects to be added to vector
        std::scoped_lock<std::mutex> add_object_lock(m_map_mutex);
        //! Generally objects should have lower resolution as there may be many more objects than
        //! background objects
        if (r_instance_image.m_score < SCORE_THRESHOLD)
        {
            spdlog::warn("Object {} with score {} is below score threshold. Not added", r_instance_image.m_label,
                         r_instance_image.m_score);
            return;
        }
        if(cv::countNonZero(r_instance_image.m_maskb) < 2500)
        {
            spdlog::warn("Object {} width score {} is too small. Not added", r_instance_image.m_label, r_instance_image.m_score);
            return;
        }
        TSDFObject::UniquePtr p_object = std::make_unique<TSDFObject>(r_frame, r_instance_image, r_camera_pose, 64);
        p_object->integrate(r_frame, r_instance_image, r_camera_pose);
        m_label_to_objects.insert(std::make_pair(r_instance_image.m_label, std::move(p_object)));
    }

    void GlobalMap::integrate_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose)
    {
        std::scoped_lock<std::mutex> integrate_background_lock(m_map_mutex);

        //! Always integrate with the last background entry in the multimap
        LabelToObjectsMap::iterator background_it = m_label_to_objects.upper_bound(0);
        if (background_it == m_label_to_objects.begin())
        {
            spdlog::error("No background object in the map");
            return;
        }
        --background_it;
        TSDFObject &background = *background_it->second;
        InstanceImage background_instance(r_frame.m_width, r_frame.m_height);

        background.integrate(r_frame, background_instance, r_camera_pose);
        spdlog::debug("Integrated background");
    }

    void GlobalMap::integrate_object(const Frame &r_frame,
                                     const InstanceImage &r_instance_image, const Eigen::Matrix4d &r_camera_pose,
                                     const Eigen::Matrix4d &r_T_maskcamera_2_camera)
    {
        //! 1. Match with requested object label
        if (r_instance_image.m_score < SCORE_THRESHOLD)
        {
            spdlog::warn("Object {} with score {} is below score threshold. Not added", r_instance_image.m_label,
                         r_instance_image.m_score);
            return;
        }
        auto label_range = m_label_to_objects.equal_range(r_instance_image.m_label);
        if (label_range.first == m_label_to_objects.end())
        {
            spdlog::warn("Requested object {} not found in map during integration, Adding new object", r_instance_image.m_label);
            add_object(r_frame, r_instance_image, r_camera_pose);
            return;
        }

        bool matched = false;
        for (auto i = label_range.first; i != label_range.second; ++i)
        {
            TSDFObject &matching_object = *i->second;
            unsigned int matching_label = i->first;

            cuda::ImageCuda<uchar, 1> src_mask;
            src_mask.Upload(r_instance_image.m_maskb);

            Eigen::Vector4d object_cent = matching_object.get_pose().block(0, 3, 4, 1);
            Eigen::Vector4d object_cent_camera_frame = r_camera_pose * object_cent;
            auto proj_object_center                  = project_point(object_cent_camera_frame.head(3), r_frame.m_intrinsic);

            //! 2. Check if the map object is in the camera frame
            if (proj_object_center[0] >= InstanceImage::BORDER_WIDTH &&
                proj_object_center[0] < r_frame.m_width - InstanceImage::BORDER_WIDTH &&
                proj_object_center[1] >= InstanceImage::BORDER_WIDTH &&
                proj_object_center[1] < r_frame.m_height - InstanceImage::BORDER_WIDTH)
            {
                //! 3. Raycast the map object to obtain the raycasted mask
                cuda::ImageCuda<float, 3> vertex, normal;
                cuda::ImageCuda<uchar, 3> color;
                vertex.Create(r_frame.m_width, r_frame.m_height);
                normal.Create(r_frame.m_width, r_frame.m_height);
                color.Create(r_frame.m_width, r_frame.m_height);
                matching_object.raycast(vertex, normal, color, r_camera_pose);
                auto color_image = color.DownloadMat();
                cv::Mat mat_gray, mat_mask;
                cv::cvtColor(color_image, mat_gray, cv::COLOR_BGR2GRAY);
                cv::threshold(mat_gray, mat_mask, 10, 255, cv::THRESH_BINARY);

                //! 4. Project the input binary mask to the current frame
                cv::Mat depthf;
                cv::rgbd::rescaleDepth(r_frame.m_depth, CV_32F, depthf);
                cuda::ImageCuda<float, 1> depth;
                depth.Upload(depthf);
                cuda::TransformCuda transform_maskcamera_to_camera;
                transform_maskcamera_to_camera.FromEigen(r_T_maskcamera_2_camera);
                cuda::PinholeCameraIntrinsicCuda intrinsic(matching_object.get_cuda_intrinsic());
                auto c_proj_mask =
                    cuda::SegmentationCuda::TransformAndProject(src_mask, depth, transform_maskcamera_to_camera, intrinsic);
                auto proj_mask = c_proj_mask.DownloadMat();
                cv::imshow("Object raycasted mask", mat_mask);
                cv::imshow("Projected mask", proj_mask);

                //! 5. Compute the intersection of projected input mask and raycasted mask
                cv::Mat intersection_mask, union_mask;
                cv::bitwise_and(proj_mask, mat_mask, intersection_mask);
                cv::bitwise_or(proj_mask, mat_mask, union_mask);
                // TODO: This should be IoU but measuring Io Projected mask
                auto quality = static_cast<float>(cv::countNonZero(intersection_mask)) /
                               static_cast<float>(cv::countNonZero(proj_mask));
                spdlog::debug("Quality of the association: {}, Source label: {}, Target label: {}", quality, r_instance_image.m_label, matching_label);

                //! 5. Finally integrate the object
                InstanceImage instance_image(proj_mask, r_instance_image.m_bbox, r_instance_image.m_label,
                                                 r_instance_image.m_score);
                if (quality > 0.2f || r_frame.m_timestamp < 20)
                {
                    matching_object.integrate(r_frame, instance_image, r_camera_pose);
                    matched = true;
                }
            }
        }
        if(!matched)
        {
            //! Else instantiate a new object
            add_object(r_frame, r_instance_image, r_camera_pose);
        }
    }

    void GlobalMap::raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                                       open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose)
    {
        std::scoped_lock<std::mutex> raycast_background_lock(m_map_mutex);
        //! Always raycast with the last background entry in the multimap
        LabelToObjectsMap::iterator background_it = m_label_to_objects.upper_bound(0);
        if (background_it == m_label_to_objects.begin())
        {
            spdlog::error("No background object in the map");
            return;
        }
        --background_it;
        TSDFObject &background = *background_it->second;
        //! TODO: Raycast layered background with objects for better vertices and normals
        background.raycast(vertex, normal, color, r_camera_pose);
        spdlog::debug("Raycasted background");
    }
}  // namespace oslam
