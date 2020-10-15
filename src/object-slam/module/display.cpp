/******************************************************************************
 * File:             display.cpp
 *
 * Author:           Akash Sharma
 * Created:          09/01/20
 * Description:      Implementation of display
 *****************************************************************************/
#include "display.h"
#include <opencv2/highgui.hpp>

#include "object-slam/payload/display_payload.h"
namespace oslam
{
    Display::Display(const std::string& window_name)
        : MISO(nullptr, "Display"),
          window_name_(window_name),
          window_3d_(window_name + " map"),
          background_color_(),
          frame_queue_("DisplayFrameQueue"),
          display_3d_queue_("Display3dQueue"),
          display_input_queue_("DisplayInputQueue")
    {
        cv::startWindowThread();

        // TODO: Enable if debug
        window_3d_.setGlobalWarnings(false);
        window_3d_.setBackgroundColor(background_color_);
        window_3d_.showWidget("Camera origin", cv::viz::WCameraPosition(0.25));
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
        cv::Mat scaled_depth;
        cv::convertScaleAbs(input_frame->depth_, scaled_depth, 0.25 * 256.0 / 1000);
        display_input->display_images_.emplace_back("Input Depth Frame", scaled_depth);

        if (curr_timestamp_ == 1)
        {
            spdlog::debug("Display input only contains input frame");
        }
        else
        {
            DisplayInput::UniquePtr display_3d;
            //! TODO: Add color map to display etc and mesh
            if (!syncQueue<Display::InputUniquePtr>(curr_timestamp_, &display_3d_queue_, &display_3d))
            {
                spdlog::error("Missing 3D display widget for requested timestamp: {}", curr_timestamp_);
                return nullptr;
            }
            if (!display_3d)
            {
                spdlog::error("Module: {} {} returned null", name_id_, display_3d_queue_.queue_id_);
            }
            display_input->display_images_.insert(display_input->display_images_.end(),
                                                  display_3d->display_images_.begin(),
                                                  display_3d->display_images_.end());

            display_input->widgets_map_ = display_3d->widgets_map_;
        }
        return display_input;
    }

    Display::OutputUniquePtr Display::runOnce(InputUniquePtr input)
    {
        const std::vector<NamedImage>& images_to_display = input->display_images_;
        const WidgetsMap& widgets_map = input->widgets_map_;
        show2dWindow(images_to_display);
        show3dWindow(widgets_map);
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

    void Display::show3dWindow(const WidgetsMap& widgets_map)
    {
        if(window_3d_.wasStopped())
        {
            //! TODO: Callback to shutdown entire pipeline!
        }
        window_3d_.removeAllWidgets();
        for(auto it = widgets_map.begin(); it != widgets_map.end(); ++it)
        {
            spdlog::debug("Showed widget: {}", it->first);
            window_3d_.showWidget(it->first, *(it->second), it->second->getPose());
        }
        window_3d_.spinOnce(1, true);
    }

    void Display::shutdownQueues()
    {
        MISOPipelineModule::shutdownQueues();
        frame_queue_.shutdown();
        display_3d_queue_.shutdown();
        display_input_queue_.shutdown();
    }

}  // namespace oslam
