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
        m_data_provider_input_queue("DataProviderInputQueue"),
        m_transport_frame_queue("TransportFrameQueue"),
        m_masked_image_queue("MaskedImageQueue")
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
    mp_data_provider->shutdown();
    mp_image_transport->shutdown();
    mp_tracker->shutdown();

    auto& global_map = GlobalMap::get_instance();
  }

  bool Controller::setup()
  {
    mp_data_reader     = std::make_shared<oslam::DataReader>(m_dataset_path);
    mp_data_provider   = std::make_shared<oslam::DataProvider>(&m_data_provider_input_queue);
    mp_image_transport = std::make_shared<oslam::ImageTransporter>(&m_transport_frame_queue, &m_masked_image_queue);
    mp_tracker         = std::make_shared<oslam::Tracker>(&m_masked_image_queue, nullptr);

    mp_data_reader->register_shutdown_callback(std::bind(&Tracker::set_max_timestamp, mp_tracker, std::placeholders::_1));

    //! DataReader fills the DataProvider Input Queue
    auto &data_provider_input_queue = m_data_provider_input_queue;
    mp_data_reader->register_provider_callback(
        [&data_provider_input_queue](Frame::UniquePtr p_frame) { data_provider_input_queue.push(std::move(p_frame)); });

    //! DataProvider fills the ImageTransporter Input Queue
    auto &transport_frame_queue = m_transport_frame_queue;
    mp_data_provider->register_output_callback([&transport_frame_queue](const Frame::Ptr &rp_frame) {
      if (rp_frame->is_maskframe())
      {
        transport_frame_queue.push(std::make_unique<Frame>(*rp_frame));
      }
    });

    //! DataProvider also fills the Tracker Frame Queue
    mp_data_provider->register_output_callback(std::bind(&Tracker::fill_frame_queue, mp_tracker, std::placeholders::_1));

    return mp_data_reader && mp_image_transport && mp_tracker;
  }

  bool Controller::shutdown_when_complete()
  {
    while (
        !m_shutdown &&
        ((mp_data_provider->is_working() ||
          (!m_data_provider_input_queue.isShutdown() && !m_data_provider_input_queue.empty())) ||
         (mp_image_transport->is_working() || (!m_transport_frame_queue.isShutdown() && !m_transport_frame_queue.empty())) ||
         (mp_tracker->is_working() || (!m_masked_image_queue.isShutdown() && !m_masked_image_queue.empty()))))
    {
      spdlog::debug(
          "m_shutdown: {}\n"
          "mp_data_provider working: {}\n"
          "m_data_provider_input_queue shutdown, empty: {}, {}\n"
          "mp_image_transport working: {}\n"
          "m_transport_frame_queue shutdown, empty: {}, {}\n"
          "mp_tracker working: {}\n"
          "m_masked_image_queue shutdown, empty: {}, {}",
          m_shutdown, mp_data_provider->is_working(), m_data_provider_input_queue.isShutdown(),
          m_data_provider_input_queue.empty(), mp_image_transport->is_working(), m_transport_frame_queue.isShutdown(),
          m_transport_frame_queue.empty(), mp_tracker->is_working(), m_masked_image_queue.isShutdown(),
          m_masked_image_queue.empty());
      std::chrono::milliseconds sleep_duration{ 500 };
      std::this_thread::sleep_for(sleep_duration);
    }
    spdlog::debug(
        "m_shutdown: {}\n"
        "mp_data_provider working: {}\n"
        "m_data_provider_input_queue shutdown, empty: {}, {}\n"
        "mp_image_transport working: {}\n"
        "m_transport_frame_queue shutdown, empty: {}, {}\n"
        "mp_tracker working: {}\n"
        "m_masked_image_queue shutdown, empty: {}, {}",
        m_shutdown, mp_data_provider->is_working(), m_data_provider_input_queue.isShutdown(),
        m_data_provider_input_queue.empty(), mp_image_transport->is_working(), m_transport_frame_queue.isShutdown(),
        m_transport_frame_queue.empty(), mp_tracker->is_working(), m_masked_image_queue.isShutdown(),
        m_masked_image_queue.empty());
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
    auto handle_provider        = std::async(std::launch::async, &oslam::DataProvider::run, mp_data_provider);
    auto handle_image_transport = std::async(std::launch::async, &oslam::ImageTransporter::run, mp_image_transport);
    auto handle_tracker         = std::async(std::launch::async, &oslam::Tracker::run, mp_tracker);
    auto handle_shutdown        = std::async(std::launch::async, &oslam::Controller::shutdown_when_complete, this);

    /* handle_shutdown.get(); */
    spdlog::info("Shutdown successful: {}", handle_shutdown.get());
    /* handle_dataset.get(); */
    spdlog::info("Dataset reader successful: {}", handle_dataset.get());
    /* handle_provider.get(); */
    spdlog::info("Dataset provider successful: {}", handle_provider.get());
    /* handle_image_transport.get(); */
    spdlog::info("Image Transporter successful: {}", handle_image_transport.get());
    bool tracker_successful = handle_tracker.get();

    spdlog::info("Tracker successful: {}", tracker_successful);
  }

}  // namespace oslam
