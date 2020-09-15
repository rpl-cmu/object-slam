/******************************************************************************
 * File:             utils.h
 *
 * Author:           Akash Sharma
 * Created:          06/16/20
 * Description:      Utility functions required in Object SLAM
 *****************************************************************************/
#ifndef OSLAM_UTILS_H
#define OSLAM_UTILS_H

#include <Open3D/Open3D.h>

#include <Eigen/Eigen>
#include <limits>

#include "object-slam/struct/instance_image.h"

namespace oslam
{
    inline Eigen::Vector2d project_point(const Eigen::Vector3d& r_point3d,
                                         const open3d::camera::PinholeCameraIntrinsic& r_intrinsic)
    {
        auto fx = r_intrinsic.intrinsic_matrix_(0, 0);
        auto fy = r_intrinsic.intrinsic_matrix_(1, 1);
        auto cx = r_intrinsic.intrinsic_matrix_(0, 2);
        auto cy = r_intrinsic.intrinsic_matrix_(1, 2);

        return Eigen::Vector2d((fx * r_point3d(0)) / r_point3d(2) + cx, (fy * r_point3d(1)) / r_point3d(2) + cy);
    }

    inline Eigen::Vector3d inverse_project_point(const Eigen::Vector2i& r_point2,
                                                 const open3d::camera::PinholeCameraIntrinsic& r_intrinsic,
                                                 float depth)
    {
        auto fx = r_intrinsic.intrinsic_matrix_(0, 0);
        auto fy = r_intrinsic.intrinsic_matrix_(1, 1);
        auto cx = r_intrinsic.intrinsic_matrix_(0, 2);
        auto cy = r_intrinsic.intrinsic_matrix_(1, 2);

        return Eigen::Vector3d((static_cast<double>(depth) * (r_point2(0) - cx)) / fx,
                               (static_cast<double>(depth) * (r_point2(1) - cy)) / fy,
                               static_cast<double>(depth));
    }

    inline bool is_valid_depth(float depth)
    {
        if (std::isnan(depth))
            return false;
        return (depth >= 0.0F) && (depth <= 4.0F);
    }

    inline bool transform_project_bbox(const BoundingBox& bbox,
                                       BoundingBox& transformed_bbox,
                                       const cv::Mat& r_depth,
                                       const open3d::camera::PinholeCameraIntrinsic& r_intrinsic,
                                       const Eigen::Matrix4d& r_transform)
    {
        Eigen::Vector2i left_top_point(bbox[0], bbox[1]);
        Eigen::Vector2i right_bottom_point(bbox[2], bbox[3]);

        float depth_left_top     = r_depth.at<float>(bbox[1], bbox[0]);
        float depth_right_bottom = r_depth.at<float>(bbox[3], bbox[2]);

        transformed_bbox = bbox;
        if (!is_valid_depth(depth_left_top) || !is_valid_depth(depth_right_bottom))
        {
            spdlog::warn("Depth at left point: {}, Depth at right point: {}\n", depth_left_top, depth_right_bottom);
            return false;
        }
        Eigen::Vector3d left_top_3dpoint =
            inverse_project_point(left_top_point, r_intrinsic, r_depth.at<float>(bbox[1], bbox[0]));
        Eigen::Vector3d right_bottom_3dpoint =
            inverse_project_point(right_bottom_point, r_intrinsic, r_depth.at<float>(bbox[3], bbox[2]));

        Eigen::Vector4d left_top_4dpoint;
        Eigen::Vector4d right_bottom_4dpoint;
        left_top_4dpoint << left_top_3dpoint(0), left_top_3dpoint(1), left_top_3dpoint(2), 1;
        right_bottom_4dpoint << right_bottom_3dpoint(0), right_bottom_3dpoint(1), right_bottom_3dpoint(2), 1;

        Eigen::Vector4d tform_left_top     = r_transform * left_top_4dpoint;
        Eigen::Vector4d tform_right_bottom = r_transform * right_bottom_4dpoint;

        Eigen::Vector2d left_top     = project_point(tform_left_top.head(3), r_intrinsic);
        Eigen::Vector2d right_bottom = project_point(tform_right_bottom.head(3), r_intrinsic);
        transformed_bbox             = BoundingBox({ int(std::round(left_top(0))),
                                         int(std::round(left_top(1))),
                                         int(std::round(right_bottom(0))),
                                         int(std::round(right_bottom(1))) });
        return true;
    }

}  // namespace oslam
#endif /* ifndef OSLAM_UTILS_H */
