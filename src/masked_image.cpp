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
  std::vector<unsigned int> labels,
  std::vector<double> scores)
  : PipelinePayload(timestamp), m_image(r_image), m_labels(std::move(labels)), m_scores(std::move(scores))
{
}

std::vector<cv::Mat> MaskedImage::process()
{
    auto masked_image = cv::Mat(m_image.width_, m_image.height_, CV_16UC1, m_image.data_.data());

    std::vector<cv::Mat> mat_masks;
    for (unsigned int i = 0; i < m_labels.size(); i++) {
        cv::Mat object_mask;
        cv::bitwise_and(masked_image, static_cast<std::uint16_t>(1u << i), object_mask);
        //! Convert to boolean object mask
        object_mask = (object_mask >= 1);
    }

    return mat_masks;
}
}// namespace oslam
