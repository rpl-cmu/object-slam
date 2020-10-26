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

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_payload.h"
#include "object-slam/utils/types.h"

namespace oslam
{
    struct InstanceImage
    {
        OSLAM_POINTER_TYPEDEFS(InstanceImage);

        static constexpr int BORDER_WIDTH = 2;
        cv::Mat maskb_;  //!< Store the float value of the object confidence in the image pixels in the mask
        Feature feature_;
        BoundingBox bbox_;  //!< Binary image which masks the rectangular box of the image
        cv::Mat bbox_mask_;
        unsigned int label_;
        double score_;

        explicit InstanceImage(const cv::Mat& mask,
                               const BoundingBox& bbox,
                               unsigned int label,
                               double score,
                               const Feature& feature = Feature())
            : maskb_(mask),
              feature_(feature),
              bbox_(bbox),
              bbox_mask_(mask.size(), CV_8UC1, cv::Scalar(0)),
              label_(label),
              score_(score)
        {
            //! Sanity shrinking
            bbox_[0] = std::min(std::max(bbox_[0], 0), mask.cols);
            bbox_[1] = std::min(std::max(bbox_[1], 0), mask.rows);
            bbox_[2] = std::min(std::max(bbox_[2], 0), mask.cols);
            bbox_[3] = std::min(std::max(bbox_[3], 0), mask.rows);

            cv::Point2i left_top     = cv::Point2i(bbox_[0], bbox_[1]);
            cv::Point2i right_bottom = cv::Point2i(bbox_[2], bbox_[3]);

            // clang-format off
            /* if ((left_top.x - 2 < mask.cols) && (left_top.x - 2 >= 0) && */
            /*     (left_top.y - 2 < mask.rows) && (left_top.y - 2 >= 0)) */
            /*         left_top -= cv::Point2i(2, 2); */
            /* if ((right_bottom.x + 2 < mask.cols) && (right_bottom.x + 2 >= 0) && */
            /*     (right_bottom.y + 2 < mask.rows) && (right_bottom.y + 2 >= 0)) */
            /*         right_bottom += cv::Point2i(2, 2); */
            // clang-format on
            bbox_mask_(cv::Rect(left_top, right_bottom)) = 255;
            bbox_mask_                                   = (bbox_mask_ >= 1);
        }

        InstanceImage(int width, int height)
            : maskb_(height, width, CV_8UC1, 1),
              feature_(),
              bbox_({ 0, 0, width, height }),
              bbox_mask_(height, width, CV_8UC1, 255),
              label_(0),
              score_(0)
        {
            maskb_ = (maskb_ >= 1);
        }
        ~InstanceImage() = default;
    };

    typedef std::vector<InstanceImage> InstanceImages;

    struct ImageTransportOutput : public PipelinePayload
    {
        OSLAM_POINTER_TYPEDEFS(ImageTransportOutput);
        InstanceImages instance_images_;

        explicit ImageTransportOutput(Timestamp timestamp) : PipelinePayload(timestamp) {}
        ~ImageTransportOutput() = default;
    };
}  // namespace oslam

#endif /* ifndef OSLAM_MASKED_IMAGE_H */
