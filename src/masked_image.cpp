/******************************************************************************
 * File:             masked_image.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Masked Image Payload implementation
 *****************************************************************************/
#include "masked_image.h"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <spdlog/spdlog.h>
#include <vector>

#include "utils/pipeline_payload.h"

namespace oslam {

MaskedImage::MaskedImage(const Timestamp timestamp,
  const open3d::geometry::Image &r_image,
  const std::vector<unsigned int> &r_labels,
  const std::vector<double> &r_scores)
  : PipelinePayload(timestamp), m_image(r_image), m_labels(r_labels), m_scores(r_scores)
{}

void MaskedImage::process_image(open3d::geometry::Image &r_color, open3d::geometry::Image &r_depth)
{
    auto masked_image = cv::Mat(m_image.width_, m_image.height_, CV_16UC1, m_image.data_.data());

    for (unsigned int i = 0; i < m_labels.size(); i++) {
        cv::Mat object_mask;
        cv::bitwise_and(masked_image, static_cast<std::uint16_t>(1 << i), object_mask);
        //! Convert to boolean object mask
        object_mask = (object_mask >= 1);

        //! TODO(Akash): Is it possible to get depth mat using a const reference to r_depth?
        auto depth_i = cv::Mat(r_depth.height_, r_depth.width_, CV_16UC1, r_depth.data_.data()).clone();
        //! Set background depth pixels to 0
        depth_i.setTo(0, ~object_mask);

        //! Write to new Open3D image
        open3d::geometry::Image o3d_depth_i;
        o3d_depth_i.Prepare(
          r_depth.width_, r_depth.height_, r_depth.num_of_channels_, r_depth.bytes_per_channel_);
        o3d_depth_i.data_.assign(depth_i.datastart, depth_i.dataend);

        mv_rgbd_objects.push_back(open3d::geometry::RGBDImage::CreateFromColorAndDepth(
          r_color, o3d_depth_i, 1000, 3.0, false));
    }
}
}// namespace oslam
