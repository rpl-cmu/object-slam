/******************************************************************************
 * File:             frame.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Frame Implementation
 *****************************************************************************/
#include "frame.h"
#include "data_reader.h"

#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>

namespace oslam {

Frame::Frame(std::size_t frame_id,
  open3d::camera::PinholeCameraIntrinsic intrinsic,
  open3d::geometry::Image color,
  open3d::geometry::Image depth,
  open3d::geometry::Image gt_mask,
  bool is_keyframe,
  const std::vector<unsigned int>& r_gt_labels,
  const std::vector<double> &r_gt_scores)
  : m_frame_id(frame_id), m_intrinsic(intrinsic), m_color(color), m_depth(depth),
    m_gt_mask(gt_mask), m_is_keyframe(is_keyframe), m_gt_labels(r_gt_labels), m_gt_scores(r_gt_scores)
{
    mp_rgbd =
      open3d::geometry::RGBDImage::CreateFromColorAndDepth(m_color, m_depth, 1000, 3.0, false);
}

std::shared_ptr<oslam::Odometry> Frame::odometry(
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

void Frame::visualize()
{
    if (!mp_pcd) mp_pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*mp_rgbd, m_intrinsic);
    open3d::visualization::DrawGeometries({ mp_pcd }, fmt::format("Frame {}", m_frame_id));
}

void Frame::process_mask(std::unique_ptr<oslam::MaskedImage> p_masked_image)
{
    m_labels.assign(p_masked_image->labels.begin(), p_masked_image->labels.end());
    m_scores.assign(p_masked_image->scores.begin(), p_masked_image->scores.end());
    auto masked_image = cv::Mat(p_masked_image->image.width_,
      p_masked_image->image.height_,
      CV_16UC1,
      p_masked_image->image.data_.data());

    for (unsigned int i = 0; i < m_labels.size(); i++) {
        cv::Mat object_mask;
        cv::bitwise_and(masked_image, static_cast<std::uint16_t>(1 << i), object_mask);
        object_mask = (object_mask >= 1);

        auto depth_i =
          cv::Mat(m_depth.height_, m_depth.width_, CV_16UC1, m_depth.data_.data()).clone();
        depth_i.setTo(0, ~object_mask);

        // Write to Open3D image
        open3d::geometry::Image o3d_depth_i;
        o3d_depth_i.Prepare(
          m_depth.width_, m_depth.height_, m_depth.num_of_channels_, m_depth.bytes_per_channel_);
        o3d_depth_i.data_.assign(depth_i.datastart, depth_i.dataend);

        mv_object_rgbd.push_back(open3d::geometry::RGBDImage::CreateFromColorAndDepth(
          m_color, o3d_depth_i, 1000, 3.0, false));
    }
}

} /* namespace oslam */
