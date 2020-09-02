/******************************************************************************
 * File:             controller.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#include "controller.h"

#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <Cuda/Common/UtilsCuda.h>

#include "object-slam/struct/map.h"

namespace oslam
{
    Controller::Controller(std::string dataset_path)
        : dataset_path_(std::move(dataset_path)),
          transport_input_queue_("TransportFrameQueue"),
          transport_output_queue_("MaskedImageQueue"),
          tracker_output_queue_("TrackerOutputQueue"),
          renderer_input_queue_("RendererInputQueue")
    {
        spdlog::debug("CONSTRUCT: Controller");
        spdlog::info("Thread ({}, {}) started", "ControllerThread", std::this_thread::get_id());

        //! TODO: Required?
        CheckCuda(cudaSetDevice(0));

        map_             = std::make_shared<oslam::Map>();
        data_reader_     = std::make_shared<oslam::DataReader>(dataset_path_);
        image_transport_ = std::make_shared<oslam::ImageTransporter>(&transport_input_queue_, &transport_output_queue_);
        tracker_         = std::make_shared<oslam::Tracker>(&tracker_output_queue_);
        renderer_        = std::make_shared<oslam::Renderer>(map_, &renderer_input_queue_);
        display_         = std::make_shared<oslam::Display>("Object SLAM");
        mapper_ =
            std::make_shared<oslam::Mapper>(map_, &tracker_output_queue_, &transport_output_queue_, &renderer_input_queue_);
    }

    bool Controller::start()
    {
        std::string figlet =
            R"(
             ____  __     _         __  ______   ___   __  ___
            / __ \/ /    (_)__ ____/ /_/ __/ /  / _ | /  |/  /
           / /_/ / _ \  / / -_) __/ __/\ \/ /__/ __ |/ /|_/ /
           \____/_.__/_/ /\__/\__/\__/___/____/_/ |_/_/  /_/
                    |___/
           )";
        std::cout << figlet << "\n";
        //! TODO: Add license
        if (setup())
        {
            run();
            return true;
        }
        return false;
    }

    void Controller::shutdown()
    {
        // TODO(Akash): Check whether the modules are running / queues are empty, etc.
        data_reader_->shutdown();
        image_transport_->shutdown();
        tracker_->shutdown();
        mapper_->shutdown();
        renderer_->shutdown();
        display_->shutdown();
    }

    bool Controller::setup()
    {
        data_reader_->registerShutdownCallback([this](Timestamp timestamp) { tracker_->setMaxTimestamp(timestamp); });
        data_reader_->registerShutdownCallback([this](Timestamp timestamp) { mapper_->setMaxTimestamp(timestamp); });
        data_reader_->registerShutdownCallback([this](Timestamp timestamp) { display_->setMaxTimestamp(timestamp); });

        //! DataReader fills the ImageTransporter Input Queue
        auto& transport_frame_queue = transport_input_queue_;
        data_reader_->registerOutputCallback([&transport_frame_queue](const Frame::Ptr& frame) {
            if (frame->is_maskframe_)
            {
                // Explicitly copy the shared frame memory
                transport_frame_queue.push(std::make_unique<Frame>(*frame));
            }
        });

        //! DataReader also fills the Tracker Frame Queue
        data_reader_->registerOutputCallback(
            [this](Frame::Ptr frame) { tracker_->fillFrameQueue(std::make_unique<Frame>(*frame)); });
        data_reader_->registerOutputCallback(
            [this](Frame::Ptr frame) { display_->fillFrameQueue(std::make_unique<Frame>(*frame)); });


        renderer_->registerOutputCallback([this](const RendererOutput::Ptr& render_payload) {
            tracker_->fillModelQueue(std::make_unique<Model>(
                render_payload->timestamp_, render_payload->colors_, render_payload->vertices_, render_payload->normals_));
        });


        return (data_reader_ && image_transport_ && tracker_ && mapper_ && renderer_);
    }

    bool Controller::shutdownWhenComplete()
    {
        // clang-format off
        while (!shutdown_ &&
                ((image_transport_->isWorking() || (!transport_input_queue_.isShutdown()  && !transport_input_queue_.empty())) ||
                 (tracker_->isWorking()         || (!transport_output_queue_.isShutdown() && !transport_output_queue_.empty())) ||
                 (mapper_->isWorking()          || (!tracker_output_queue_.isShutdown()   && !tracker_output_queue_.empty())) ||
                 (renderer_->isWorking()        || (!renderer_input_queue_.isShutdown()   && !renderer_input_queue_.empty()))
                )
              )
        {
            spdlog::debug("\n"
                "shutdown_                           : {}\n"
                "image_transport_ working            : {}\n"
                "transport_input_queue_ shutdown     : {}\n"
                "transport_input_queue_ empty        : {}\n"
                "tracker_ working                    : {}\n"
                "transport_output_queue_ shutdown    : {}\n"
                "transport_output_queue_ empty       : {}\n"
                "mapper_ working                     : {}\n"
                "tracker_output_queue_ shutdown      : {}\n"
                "tracker_output_queue_ empty         : {}",
                shutdown_,
                image_transport_->isWorking(),
                transport_input_queue_.isShutdown(),
                transport_input_queue_.empty(),
                tracker_->isWorking(),
                transport_output_queue_.isShutdown(),
                transport_output_queue_.empty(),
                mapper_->isWorking(),
                tracker_output_queue_.isShutdown(),
                tracker_output_queue_.empty());

            static constexpr int half_s = 500;
            std::chrono::milliseconds sleep_duration{ half_s };
            std::this_thread::sleep_for(sleep_duration);
        }
            spdlog::debug("\n"
                "shutdown_                           : {}\n"
                "image_transport_ working            : {}\n"
                "transport_input_queue_ shutdown     : {}\n"
                "transport_input_queue_ empty        : {}\n"
                "tracker_ working                    : {}\n"
                "transport_output_queue_ shutdown    : {}\n"
                "transport_output_queue_ empty       : {}\n"
                "mapper_ working                     : {}\n"
                "tracker_output_queue_ shutdown      : {}\n"
                "tracker_output_queue_ empty         : {}",
                shutdown_,
                image_transport_->isWorking(),
                transport_input_queue_.isShutdown(),
                transport_input_queue_.empty(),
                tracker_->isWorking(),
                transport_output_queue_.isShutdown(),
                transport_output_queue_.empty(),
                mapper_->isWorking(),
                tracker_output_queue_.isShutdown(),
                tracker_output_queue_.empty());
        // clang-format on
        if (!shutdown_)
        {
            shutdown();
            shutdown_ = true;
        }
        return true;
    }

    void Controller::run()
    {
        // Start thread for each component
        auto handle_dataset         = std::async(std::launch::async, &oslam::DataReader::run, data_reader_);
        auto handle_image_transport = std::async(std::launch::async, &oslam::ImageTransporter::run, image_transport_);
        auto handle_tracker         = std::async(std::launch::async, &oslam::Tracker::run, tracker_);
        auto handle_mapper          = std::async(std::launch::async, &oslam::Mapper::run, mapper_);
        auto handle_renderer        = std::async(std::launch::async, &oslam::Renderer::run, renderer_);
        auto handle_shutdown        = std::async(std::launch::async, &oslam::Controller::shutdownWhenComplete, this);

        //! TODO: Should this be asynchronously called?
        display_->run();
        /* auto handle_display = std::async(std::launch::deferred, &oslam::Display::run, display_); */

        handle_shutdown.get();
        spdlog::info("Shutdown successful: {}", handle_shutdown.get());
        handle_dataset.get();
        spdlog::info("Dataset reader successful: {}", handle_dataset.get());
        handle_image_transport.get();
        spdlog::info("Image Transporter successful: {}", handle_image_transport.get());
        bool tracker_successful = handle_tracker.get();
        spdlog::info("Tracker successful: {}", tracker_successful);
        bool mapper_successful = handle_mapper.get();
        spdlog::info("Mapper successful: {}", mapper_successful);
        bool renderer_successful = handle_renderer.get();
        spdlog::info("Renderer successful: {}", renderer_successful);
    }

}  // namespace oslam
