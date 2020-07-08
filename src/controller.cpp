/******************************************************************************
 * File:             controller.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#include "controller.h"

#include <Cuda/Common/UtilsCuda.h>

#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "map.h"

namespace oslam
{
    Controller::Controller(const std::map<std::string, docopt::value> &r_args)
        : m_visualize(r_args.at("--vis")),
          m_debug(r_args.at("--debug")),
          m_dataset_path(r_args.at("<dataset_path>").asString()),
          m_transport_input_queue("TransportFrameQueue"),
          m_transport_output_queue("MaskedImageQueue"),
          m_tracker_output_queue("TrackerOutputQueue"),
          m_renderer_input_queue("RendererInputQueue"),
          m_renderer_output_queue("RendererOutputQueue")
    {
        if (m_debug)
        {
            spdlog::set_level(spdlog::level::debug);
        }
        spdlog::info("m_debug, m_visualize: ({}, {})", m_debug, m_visualize);
        spdlog::info("Thread ({}, {}) started", "ControllerThread", std::this_thread::get_id());
    }

    int Controller::start()
    {
        if (setup())
        {
            run();
            return 0;
        }
        return -1;
    }

    void Controller::shutdown()
    {
        // TODO(Akash): Check whether the modules are running / queues are empty, etc.
        mp_data_reader->shutdown();
        mp_image_transport->shutdown();
        mp_tracker->shutdown();
        mp_mapper->shutdown();
        mp_renderer->shutdown();
    }

    bool Controller::setup()
    {
        mp_data_reader     = std::make_shared<oslam::DataReader>(m_dataset_path);
        mp_image_transport = std::make_shared<oslam::ImageTransporter>(&m_transport_input_queue, &m_transport_output_queue);
        mp_tracker         = std::make_shared<oslam::Tracker>(&m_renderer_output_queue, &m_tracker_output_queue);
        mp_mapper          = std::make_shared<oslam::Mapper>(&m_tracker_output_queue, &m_transport_output_queue, &m_renderer_input_queue);
        mp_renderer        = std::make_shared<oslam::Renderer>(&m_renderer_input_queue, &m_renderer_output_queue);

        mp_data_reader->register_shutdown_callback(
            std::bind(&Tracker::set_max_timestamp, mp_tracker, std::placeholders::_1));

        mp_data_reader->register_shutdown_callback(
            std::bind(&Mapper::set_max_timestamp, mp_mapper, std::placeholders::_1));

        //! DataReader fills the ImageTransporter Input Queue
        auto &transport_frame_queue = m_transport_input_queue;
        mp_data_reader->register_output_callback([&transport_frame_queue](Frame::Ptr p_frame) {
            if (p_frame->m_is_maskframe)
                transport_frame_queue.push(std::make_unique<Frame>(*p_frame));
        });
        //! DataReader also fills the Tracker Frame Queue
        mp_data_reader->register_output_callback(std::bind(&Tracker::fill_frame_queue, mp_tracker, std::placeholders::_1));

        return mp_data_reader && mp_image_transport && mp_tracker && mp_mapper;
    }

    bool Controller::shutdown_when_complete()
    {
        // clang-format off
        while (!m_shutdown &&
                ((mp_image_transport->is_working() || (!m_transport_input_queue.isShutdown()  && !m_transport_input_queue.empty())) ||
                 (mp_tracker->is_working()         || (!m_transport_output_queue.isShutdown() && !m_transport_output_queue.empty())) ||
                 (mp_mapper->is_working()          || (!m_tracker_output_queue.isShutdown()   && !m_tracker_output_queue.empty())) ||
                 (mp_renderer->is_working()        || (!m_renderer_input_queue.isShutdown() && !m_renderer_input_queue.empty())) ||
                 (!m_renderer_output_queue.isShutdown() && !m_renderer_output_queue.empty())
                )
              )
        {
            spdlog::debug("\n"
                "m_shutdown                           : {}\n"
                "mp_image_transport working           : {}\n"
                "m_transport_input_queue shutdown     : {}\n"
                "m_transport_input_queue empty        : {}\n"
                "mp_tracker working                   : {}\n"
                "m_transport_output_queue shutdown    : {}\n"
                "m_transport_output_queue empty       : {}\n"
                "mp_mapper working                    : {}\n"
                "m_tracker_output_queue shutdown      : {}\n"
                "m_tracker_output_queue empty         : {}",
                m_shutdown,
                mp_image_transport->is_working(),
                m_transport_input_queue.isShutdown(),
                m_transport_input_queue.empty(),
                mp_tracker->is_working(),
                m_transport_output_queue.isShutdown(),
                m_transport_output_queue.empty(),
                mp_mapper->is_working(),
                m_tracker_output_queue.isShutdown(),
                m_tracker_output_queue.empty());

            std::chrono::milliseconds sleep_duration{ 500 };
            std::this_thread::sleep_for(sleep_duration);
        }
            spdlog::debug("\n"
                "m_shutdown                           : {}\n"
                "mp_image_transport working           : {}\n"
                "m_transport_input_queue shutdown     : {}\n"
                "m_transport_input_queue empty        : {}\n"
                "mp_tracker working                   : {}\n"
                "m_transport_output_queue shutdown    : {}\n"
                "m_transport_output_queue empty       : {}\n"
                "mp_mapper working                    : {}\n"
                "m_tracker_output_queue shutdown      : {}\n"
                "m_tracker_output_queue empty         : {}",
                m_shutdown,
                mp_image_transport->is_working(),
                m_transport_input_queue.isShutdown(),
                m_transport_input_queue.empty(),
                mp_tracker->is_working(),
                m_transport_output_queue.isShutdown(),
                m_transport_output_queue.empty(),
                mp_mapper->is_working(),
                m_tracker_output_queue.isShutdown(),
                m_tracker_output_queue.empty());
        // clang-format on

        if (!m_shutdown)
        {
            shutdown();
            m_shutdown = true;
        }
        return true;
    }

    void Controller::run()
    {
        // Start thread for each component
        auto handle_dataset         = std::async(std::launch::async, &oslam::DataReader::run, mp_data_reader);
        auto handle_image_transport = std::async(std::launch::async, &oslam::ImageTransporter::run, mp_image_transport);
        auto handle_tracker         = std::async(std::launch::async, &oslam::Tracker::run, mp_tracker);
        auto handle_mapper          = std::async(std::launch::async, &oslam::Mapper::run, mp_mapper);
        auto handle_renderer        = std::async(std::launch::async, &oslam::Renderer::run, mp_renderer);
        auto handle_shutdown        = std::async(std::launch::async, &oslam::Controller::shutdown_when_complete, this);

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
