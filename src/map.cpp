/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Open3D/Camera/PinholeCameraIntrinsic.h>
#include <opencv2/core/hal/interface.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

namespace oslam
{

  void GlobalMap::add_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose)
  {
    using namespace open3d;
    //! Don't allow other objects to be added to vector
    std::scoped_lock<std::mutex> add_background_lock(m_map_mutex);
    TSDFObject::UniquePtr p_background = std::make_unique<TSDFObject>(r_frame, 0, 100.0, r_camera_pose, 256);
    cv::Mat mask                       = cv::Mat::ones(r_frame.get_depth().height_, r_frame.get_depth().width_, CV_8U);
    mv_objects.push_back(std::move(p_background));
  }

  void GlobalMap::add_object(const Frame &r_frame, const cv::Mat &r_mask, unsigned int label, double score,
                             const Eigen::Matrix4d &r_camera_pose)
  {
    //! Don't allow other objects to be added to vector
    std::scoped_lock<std::mutex> add_object_lock(m_map_mutex);

    //! Generally objects should have lower resolution as there may be many more objects than
    //! background objects
    TSDFObject::UniquePtr p_object = std::make_unique<TSDFObject>(r_frame, label, score, r_camera_pose, 64);

    //! Generally during integration the camera pose will be different than the original object
    //! volume
    //! TODO: remove intrinsics
    p_object->integrate(r_frame, r_mask, r_camera_pose);
    mv_objects.push_back(std::move(p_object));
  }

  void GlobalMap::integrate_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose)
  {
    int i = 0;
    //! Change this to hashtable?
    for (; i < mv_objects.size(); i++)
      if (mv_objects.at(i)->is_background())
        break;

    if (!mv_objects.at(i))
    {
      spdlog::error("Background object does not exist!!");
    }
    TSDFObject &background = *mv_objects.at(i);

    cv::Mat mask = cv::Mat::ones(r_frame.get_depth().height_, r_frame.get_depth().width_, CV_8U);
    mask         = (mask >= 0);
    //! TODO(Remove)
    background.integrate(r_frame, mask, r_camera_pose);
  }

  void GlobalMap::raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                                     open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose)
  {
    unsigned int i = 0;
    for (; i < mv_objects.size(); i++)
    {
      if (mv_objects.at(i)->is_background())
        break;
    }

    if (!mv_objects.at(i))
    {
      spdlog::error("Background object does not exist!!");
    }
    TSDFObject &background = *mv_objects.at(i);
    background.raycast(vertex, normal, color, r_camera_pose);
  }
}  // namespace oslam
