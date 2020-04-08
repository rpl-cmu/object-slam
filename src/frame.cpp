/******************************************************************************
 * File:             frame.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Frame Implementation
 *****************************************************************************/
#include "frame.h"
#include "dataset.h"

#include <Open3D/Odometry/Odometry.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>


oslam::Frame::Frame(std::size_t index,
  const oslam::RGBDdata &r_data,
  open3d::camera::PinholeCameraIntrinsic intrinsic)
  : m_frame_id(index), m_color(r_data.color), m_depth(r_data.depth),
    m_intrinsic(std::move(intrinsic)), m_pose(Eigen::Matrix4d::Identity())
{
    mp_rgbd =
      open3d::geometry::RGBDImage::CreateFromColorAndDepth(m_color, m_depth, 1000, 3.0, false);
}

oslam::Frame::~Frame() {}

std::shared_ptr<oslam::Odometry> oslam::Frame::odometry(
  [[maybe_unused]] std::shared_ptr<Frame> p_target_frame,
  const Eigen::Matrix4d &r_init_transformation /* = Identity() */)
{
    using namespace open3d::odometry;
    std::shared_ptr<Odometry> p_rgbd_odometry(new Odometry());
    if (p_target_frame) {
        bool success = false;
        spdlog::debug("Source Image size: ({}, {})",
          mp_rgbd->color_.num_of_channels_,
          mp_rgbd->color_.bytes_per_channel_);
        spdlog::debug("Target image channels: ({}, {})",
          p_target_frame->get_rgbd()->color_.num_of_channels_,
          p_target_frame->get_rgbd()->color_.bytes_per_channel_);
        std::tie(success, p_rgbd_odometry->transform, p_rgbd_odometry->information) =
          ComputeRGBDOdometry(
            *mp_rgbd, *p_target_frame->get_rgbd(), m_intrinsic, r_init_transformation);
        if (success) return p_rgbd_odometry;
    }
    return p_rgbd_odometry;
}

void oslam::Frame::visualize()
{
    if (!mp_pcd) mp_pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*mp_rgbd, m_intrinsic);
    open3d::visualization::DrawGeometries({ mp_pcd }, fmt::format("Frame {}", m_frame_id));
}
