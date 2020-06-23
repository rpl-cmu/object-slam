/******************************************************************************
 * File:             masked_image.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Masked Image payload
 *****************************************************************************/
#ifndef OSLAM_MASKED_IMAGE_H
#define OSLAM_MASKED_IMAGE_H

#include <Open3D/Open3D.h>
#include <spdlog/spdlog.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/macros.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
    using BoundingBox = std::array<float, 4>;
    struct InstanceImage
    {
        OSLAM_POINTER_TYPEDEFS(InstanceImage);

        static constexpr int BORDER_WIDTH = 20;
        //! Store the float value of the object confidence in the image pixels in the mask
        cv::Mat m_mask;
        cv::Mat m_maskb;
        //! Binary image which masks the rectangular box of the image
        BoundingBox m_bbox;
        cv::Mat m_bbox_mask;
        //! Object label and score
        unsigned int m_label;
        double m_score;

        explicit InstanceImage(const cv::Mat& mask, const BoundingBox& bbox, unsigned int label, double score)
            : m_mask(mask.size(), CV_32FC1, cv::Scalar(1 - 0.99)),
              m_maskb(mask),
              m_bbox(bbox),
              m_bbox_mask(mask.size(), CV_8UC1, cv::Scalar(0)),
              m_label(label),
              m_score(score)
        {
            m_bbox_mask(cv::Rect(cv::Point2i(int(bbox[0]), int(bbox[1])), cv::Point2i(int(bbox[2]), int(bbox[3])))) = 255;
            m_bbox_mask = (m_bbox_mask >= 1);
            m_mask.setTo(0.99, mask);
        }

        InstanceImage(int width, int height)
            : m_mask(height, width, CV_32FC1, 1),
              m_maskb(height, width, CV_8UC1, 1),
              m_bbox({ 0, 0, static_cast<float>(width), static_cast<float>(height) }),
              m_bbox_mask(height, width, CV_8UC1, 255),
              m_label(0),
              m_score(0)
        {
            m_maskb = (m_maskb >= 1);
        }

        ~InstanceImage() = default;
    };

    struct ImageTransportOutput : public PipelinePayload
    {
        OSLAM_POINTER_TYPEDEFS(ImageTransportOutput);
        std::vector<InstanceImage> m_instance_images;

        explicit ImageTransportOutput(Timestamp timestamp) : PipelinePayload(timestamp) {}
        ~ImageTransportOutput() = default;
    };
}  // namespace oslam

#endif /* ifndef OSLAM_MASKED_IMAGE_H */
