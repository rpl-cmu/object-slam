/******************************************************************************
* File:             display_payload.h
*
* Author:           Akash Sharma
* Created:          09/02/20
* Description:      Input payload for Display
*****************************************************************************/
#ifndef OSLAM_DISPLAY_PAYLOAD_H
#define OSLAM_DISPLAY_PAYLOAD_H

#include <opencv2/opencv.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <utility>

#include "object-slam/utils/pipeline_payload.h"

namespace oslam {

    struct NamedImage
    {
        NamedImage(std::string  name, cv::Mat image) : name_(std::move(name)), image_(std::move(image)) {}
        ~NamedImage() = default;
        std::string name_;
        cv::Mat image_;
    };

    using WidgetPtr = std::unique_ptr<cv::viz::Viz3d>;

    struct NamedWidget
    {
        std::string name_;
        WidgetPtr widget_;
    };


    /*! \class DisplayInput : public PipelinePayload
     *  \brief Brief class description
     *
     *  Detailed description
     */
    struct DisplayInput : public PipelinePayload
    {
    public:
        explicit DisplayInput(Timestamp timestamp) :
            PipelinePayload(timestamp), camera_pose_(cv::Affine3d::Identity()) {};
        ~DisplayInput() override = default;

    public:
        std::vector<NamedImage> display_images_; /*!< Member description */
        std::vector<NamedWidget> display_widgets_;
        cv::Affine3d camera_pose_;
    };
}
#endif /* ifndef OSLAM_DISPLAY_PAYLOAD_H */

