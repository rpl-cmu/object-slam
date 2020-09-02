/******************************************************************************
 * File:             display.cpp
 *
 * Author:           Akash Sharma
 * Created:          09/01/20
 * Description:      Implementation of display
 *****************************************************************************/
#include "display.h"
#include <opencv2/highgui.hpp>

namespace oslam
{
    Display::Display(const std::string& window_name)
        : MISO(nullptr, "Display"),
          window_name_(window_name),
          window_3d_(window_name + " map"),
          background_color_(cv::viz::Color::white()),
          frame_queue_("DisplayFrameQueue"),
          render_queue_("DisplayRenderQueue")
    {
        cv::startWindowThread();

        // TODO: Enable if debug
        window_3d_.setGlobalWarnings(false);
        window_3d_.setBackgroundColor(background_color_);
        window_3d_.showWidget("Coordinate widget", cv::viz::WCoordinateSystem());
        window_3d_.setViewerPose(cv::Affine3d::Identity());
    }

    Display::~Display() { cv::destroyWindow(window_name_); }

    Display::InputUniquePtr Display::getInputPacket()
    {
        Frame::UniquePtr input_frame;
        bool queue_state = frame_queue_.popBlocking(input_frame);
        if (!input_frame || !queue_state)
        {
            spdlog::error("Module: {} {} returned null", name_id_, frame_queue_.queue_id_);
            return nullptr;
        }
        curr_timestamp_ = input_frame->timestamp_;

        Display::InputUniquePtr display_input;

        display_input = std::make_unique<DisplayInput>(curr_timestamp_);
        display_input->display_images_.emplace_back("Input Color Frame", input_frame->color_);
        display_input->display_images_.emplace_back("Input Depth Frame", input_frame->depth_);

        if (curr_timestamp_ == 1)
        {
            spdlog::debug("Display input only contains input frame");
        }
        else
        {
            //! TODO: Add color map to display etc and mesh
        }
        return display_input;
    }

    Display::OutputUniquePtr Display::runOnce(InputUniquePtr input)
    {
        const std::vector<NamedImage>& images_to_display = input->display_images_;
        show2dWindow(images_to_display);
        spdlog::info("Showed window");
        return nullptr;
    }

    void Display::show2dWindow(const std::vector<NamedImage>& images_to_display) const
    {
        for (const auto& curr_img : images_to_display)
        {
            cv::namedWindow(curr_img.name_);
            cv::imshow(curr_img.name_, curr_img.image_);
        }
    }

    void Display::shutdownQueues()
    {
        MISOPipelineModule::shutdownQueues();
        frame_queue_.shutdown();
        render_queue_.shutdown();
    }

}  // namespace oslam
