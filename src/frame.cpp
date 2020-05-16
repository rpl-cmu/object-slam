/******************************************************************************
 * File:             frame.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Frame Implementation
 *****************************************************************************/
#include "frame.h"
#include "data_reader.h"
#include "utils/pipeline_payload.h"

#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>

namespace oslam {

Frame::Frame(std::size_t frame_id,
  open3d::camera::PinholeCameraIntrinsic intrinsic,
  const open3d::geometry::Image &r_color,
  const open3d::geometry::Image &r_depth,
  bool is_maskframe,
  const open3d::geometry::Image &r_gt_mask,
  const std::vector<unsigned int> &r_gt_labels,
  const std::vector<double> &r_gt_scores)
  : PipelinePayload(frame_id), m_intrinsic(intrinsic), m_color(r_color), m_depth(r_depth),
    m_is_maskframe(is_maskframe), m_gt_mask(frame_id, r_gt_mask, r_gt_labels, r_gt_scores)
{
    mp_rgbd =
      open3d::geometry::RGBDImage::CreateFromColorAndDepth(m_color, m_depth, 1000, 3.0, false);
}

//!TODO(Akash): Reconsider when required
/* std::shared_ptr<oslam::Odometry> Frame::odometry( */
/*   [[maybe_unused]] std::shared_ptr<Frame> p_target_frame, */
/*   const Eigen::Matrix4d &r_init_transformation /1* = Identity() *1/) */
/* { */
/*     using namespace open3d::odometry; */
/*     std::shared_ptr<Odometry> p_rgbd_odometry(new Odometry()); */
/*     if (p_target_frame) { */
/*         bool success = false; */
/*         spdlog::debug("Source Image size: ({}, {})", */
/*           mp_rgbd->color_.num_of_channels_, */
/*           mp_rgbd->color_.bytes_per_channel_); */
/*         spdlog::debug("Target image channels: ({}, {})", */
/*           p_target_frame->get_rgbd()->color_.num_of_channels_, */
/*           p_target_frame->get_rgbd()->color_.bytes_per_channel_); */
/*         std::tie(success, p_rgbd_odometry->transform, p_rgbd_odometry->information) = */
/*           ComputeRGBDOdometry( */
/*             *mp_rgbd, *p_target_frame->get_rgbd(), m_intrinsic, r_init_transformation); */
/*         if (success) return p_rgbd_odometry; */
/*     } */
/*     return p_rgbd_odometry; */
/* } */
/* void Frame::visualize() */
/* { */
/*     //! TODO: Reconsider visualization for frame later */
/*     if (!mp_pcd) mp_pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*mp_rgbd, m_intrinsic); */
/*     open3d::visualization::DrawGeometries({ mp_pcd }, fmt::format("Frame {}", m_frame_id)); */
/* } */
} // namespace oslam
