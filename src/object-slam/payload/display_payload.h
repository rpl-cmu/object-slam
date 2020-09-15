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

namespace oslam
{
    struct NamedImage
    {
        NamedImage(std::string name, cv::Mat image) : name_(std::move(name)), image_(std::move(image)) {}
        ~NamedImage() = default;
        std::string name_;
        cv::Mat image_;
    };

    using WidgetPtr  = std::shared_ptr<cv::viz::Widget3D>;
    using WidgetsMap = std::map<std::string, WidgetPtr>;

    /*! \class DisplayInput : public PipelinePayload
     *  \brief Brief class description
     *
     *  Detailed description
     */
    struct DisplayInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(DisplayInput);

        explicit DisplayInput(Timestamp timestamp) : PipelinePayload(timestamp) {}

        DisplayInput(Timestamp timestamp, const std::vector<NamedImage>& display_images, const WidgetsMap& widgets_map)
            : PipelinePayload(timestamp), display_images_(display_images), widgets_map_(widgets_map)
        {
        }
        ~DisplayInput() override = default;

       public:
        std::vector<NamedImage> display_images_;
        WidgetsMap widgets_map_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_DISPLAY_PAYLOAD_H */
