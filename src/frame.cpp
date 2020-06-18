/******************************************************************************
 * File:             frame.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Frame Implementation
 *****************************************************************************/
#include "frame.h"

#include <Open3D/Open3D.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "data_reader.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
  Frame::Frame(std::size_t frame_id, const open3d::camera::PinholeCameraIntrinsic &r_intrinsic,
               const open3d::geometry::Image &r_color, const open3d::geometry::Image &r_depth, bool is_maskframe,
               const open3d::geometry::Image &r_gt_mask, const std::vector<unsigned int> &r_gt_labels,
               const std::vector<double> &r_gt_scores)
      : PipelinePayload(frame_id),
        m_intrinsic(r_intrinsic),
        m_color(r_color),
        m_depth(r_depth),
        m_is_maskframe(is_maskframe),
        m_gt_mask(frame_id, r_gt_mask, r_gt_labels, r_gt_scores)
  {
    mp_rgbd     = open3d::geometry::RGBDImage::CreateFromColorAndDepth(m_color, m_depth, 1000, 3.0, false);

    cv::Mat color = cv::Mat(m_color.height_, m_color.width_, CV_8UC3, m_color.data_.data());
    cv::cvtColor(color, m_color_mat, cv::COLOR_RGB2BGR);
    m_depth_mat = cv::Mat(m_depth.height_, m_depth.width_, CV_32FC1, m_depth.data_.data());
  }
}  // namespace oslam
