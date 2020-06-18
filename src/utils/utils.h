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

namespace oslam
{
  Eigen::Vector2d project_point(const Eigen::Vector3d& r_point3d, const open3d::camera::PinholeCameraIntrinsic& r_intrinsic)
  {
    auto fx = r_intrinsic.intrinsic_matrix_(0, 0);
    auto fy = r_intrinsic.intrinsic_matrix_(1, 1);
    auto cx = r_intrinsic.intrinsic_matrix_(0, 2);
    auto cy = r_intrinsic.intrinsic_matrix_(1, 2);

    return Eigen::Vector2d((fx * r_point3d(0)) / r_point3d(2) + cx, (fy * r_point3d(1)) / r_point3d(2) + cy);
  }

  Eigen::Vector3d inverse_project_point(const Eigen::Vector2i& r_point2,
                                        const open3d::camera::PinholeCameraIntrinsic& r_intrinsic, float depth)
  {
    auto fx = r_intrinsic.intrinsic_matrix_(0, 0);
    auto fy = r_intrinsic.intrinsic_matrix_(1, 1);
    auto cx = r_intrinsic.intrinsic_matrix_(0, 2);
    auto cy = r_intrinsic.intrinsic_matrix_(1, 2);

    return Eigen::Vector3d((static_cast<double>(depth) * (r_point2(0) - cx)) / fx, (static_cast<double>(depth) * (r_point2(1) - cy)) / fy, depth);
  }
}  // namespace oslam
#endif /* ifndef OSLAM_UTILS_H */
