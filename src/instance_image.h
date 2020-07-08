/******************************************************************************
 * File:             masked_image.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/5
 * Description:      Masked Image payload
 *****************************************************************************/
#ifndef OSLAM_MASKED_IMAGE_H
#define OSLAM_MASKED_IMAGE_H

#include <Open3D/Open3D.h>
#include <spdlog/spdlog.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/types.h"
#include "utils/macros.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
    struct InstanceImage
    {
        OSLAM_POINTER_TYPEDEFS(InstanceImage);

        static constexpr int BORDER_WIDTH = 10;
        //! Store the float value of the object confidence in the image pixels in the mask
        cv::Mat m_maskb;
        //! Binary image which masks the rectangular box of the image
        BoundingBox m_bbox;
        cv::Mat m_bbox_mask;
        //! Object label and score
        unsigned int m_label;
        double m_score;

        explicit InstanceImage(const cv::Mat& mask, const BoundingBox& bbox, unsigned int label, double score)
            : m_maskb(mask), m_bbox(bbox), m_bbox_mask(mask.size(), CV_8UC1, cv::Scalar(0)), m_label(label), m_score(score)
        {
            //! Sanity shrinking
            m_bbox[0] = std::min(std::max(m_bbox[0], 0), mask.cols);
            m_bbox[1] = std::min(std::max(m_bbox[1], 0), mask.rows);
            m_bbox[2] = std::min(std::max(m_bbox[2], 0), mask.cols);
            m_bbox[3] = std::min(std::max(m_bbox[3], 0), mask.rows);

            cv::Point2i left_top     = cv::Point2i(m_bbox[0], m_bbox[1]);
            cv::Point2i right_bottom = cv::Point2i(m_bbox[2], m_bbox[3]);


            //! This ensures that subvolumes near edges are allocated appropriately
            if ((left_top.x - 5 < mask.cols) && (left_top.x - 5 >= 0) && (left_top.y - 5 < mask.rows) &&
                (left_top.y - 5 >= 0))
                left_top -= cv::Point2i(5, 5);
            if ((right_bottom.x + 5 < mask.cols) && (right_bottom.x + 5 >= 0) && (right_bottom.y + 5 < mask.rows) &&
                (right_bottom.y + 5 >= 0))
                right_bottom += cv::Point2i(5, 5);
            m_bbox_mask(cv::Rect(left_top, right_bottom)) = 255;
            m_bbox_mask                                   = (m_bbox_mask >= 1);
        }

        InstanceImage(int width, int height)
            : m_maskb(height, width, CV_8UC1, 1),
              m_bbox({ 0, 0, width, height }),
              m_bbox_mask(height, width, CV_8UC1, 255),
              m_label(0),
              m_score(0)
        {
            m_maskb = (m_maskb >= 1);
        }

        ~InstanceImage() = default;
    };

    typedef std::vector<InstanceImage> InstanceImages;

    struct ImageTransportOutput : public PipelinePayload
    {
        OSLAM_POINTER_TYPEDEFS(ImageTransportOutput);
        InstanceImages m_instance_images;

        explicit ImageTransportOutput(Timestamp timestamp) : PipelinePayload(timestamp) {}
        ~ImageTransportOutput() = default;
    };
}  // namespace oslam

#endif /* ifndef OSLAM_MASKED_IMAGE_H */
