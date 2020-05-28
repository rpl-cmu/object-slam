/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <Open3D/Camera/PinholeCameraIntrinsic.h>
#include <opencv2/core/hal/interface.h>
#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>

namespace oslam {

void GlobalMap::add_background(open3d::geometry::Image &r_color,
  open3d::geometry::Image &r_depth,
  open3d::cuda::PinholeCameraIntrinsicCuda &r_intrinsic,
  open3d::camera::PinholeCameraIntrinsic intrinsics,
  Eigen::Matrix4d &r_camera_pose)
{
    //! Don't allow other objects to be added to vector
    std::scoped_lock<std::mutex> add_background_lock(m_map_mutex);
    TSDFObject::UniquePtr p_background =
      std::make_unique<TSDFObject>(r_color, r_depth, 0, 100.0, r_intrinsic, r_camera_pose, 256);

    cv::Mat mask = cv::Mat::ones(r_depth.height_, r_depth.width_, CV_8U);
    mask = (mask >= 0);

    spdlog::debug("Integrating first frame into volume");
    p_background->integrate(r_color, r_depth, mask, intrinsics, r_camera_pose);
    mv_objects.push_back(std::move(p_background));
}
void GlobalMap::add_object(open3d::geometry::Image &r_color,
  open3d::geometry::Image &r_depth,
  cv::Mat &r_mask,
  unsigned int label,
  double score,
  open3d::cuda::PinholeCameraIntrinsicCuda &r_intrinsic,
  Eigen::Matrix4d &r_camera_pose)
{
    //! Don't allow other objects to be added to vector
    std::scoped_lock<std::mutex> add_object_lock(m_map_mutex);

    //! Generally objects should have lower resolution as there may be many more objects than
    //! background objects
    TSDFObject::UniquePtr p_object =
      std::make_unique<TSDFObject>(r_color, r_depth, label, score, r_intrinsic, r_camera_pose, 64);

    //! Generally during integration the camera pose will be different than the original object
    //! volume
    //! TODO: remove intrinsics
    p_object->integrate(r_color, r_depth, r_mask, open3d::camera::PinholeCameraIntrinsic(), r_camera_pose);
    mv_objects.push_back(std::move(p_object));
}

void GlobalMap::integrate_background(open3d::geometry::Image &r_color,
  open3d::geometry::Image &r_depth,
  Eigen::Matrix4d camera_pose)
{

    int i = 0;
    //! Change this to hashtable?
    for (; i < mv_objects.size(); i++)
        if (mv_objects.at(i)->is_background()) break;

    if (!mv_objects.at(i)) { spdlog::error("Background object does not exist!!"); }
    TSDFObject &background = *mv_objects.at(i);

    cv::Mat mask = cv::Mat::ones(r_depth.height_, r_depth.width_, CV_8U);
    mask = (mask >= 0);
    //! TODO(Remove)
    background.integrate(r_color, r_depth, mask, open3d::camera::PinholeCameraIntrinsic(), camera_pose);
}

void GlobalMap::raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex,
  open3d::cuda::ImageCuda<float, 3> &normal,
  open3d::cuda::ImageCuda<uchar, 3> &color,
  open3d::cuda::PinholeCameraIntrinsicCuda intrinsic,
  Eigen::Matrix4d camera_pose)
{

    int i = 0;
    for (; i < mv_objects.size(); i++)
        if (mv_objects.at(i)->is_background()) break;

    if (!mv_objects.at(i)) { spdlog::error("Background object does not exist!!"); }
    TSDFObject &background = *mv_objects.at(i);
    background.raycast(vertex, normal, color, intrinsic, camera_pose);
}
}// namespace oslam
