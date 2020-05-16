/******************************************************************************
 * File:             controller.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#include "controller.h"
#include <memory>
#include <vector>
#include <future>

#include "map.h"

namespace oslam {

Controller::Controller(const std::map<std::string, docopt::value> &r_args)
  : m_visualize(r_args.at("--vis")), m_debug(r_args.at("--debug")),
    m_dataset_path(r_args.at("<dataset_path>").asString()),
    m_data_provider_input_queue("DataProviderInputQueue"),
    m_transport_frame_queue("TransportFrameQueue"), m_masked_image_queue("MaskedImageQueue")
{
    if (m_debug) { spdlog::set_level(spdlog::level::debug); }
    spdlog::info("m_debug, m_visualize: ({}, {})", m_debug, m_visualize);
    spdlog::info("Thread ({}, {}) started", "ControllerThread", std::this_thread::get_id());
}

int Controller::start()
{
    if (setup()) {
        run();
        return 0;
    }
    return -1;
}

bool Controller::setup()
{
    mp_data_reader = std::make_shared<oslam::DataReader>(m_dataset_path);
    mp_data_provider = std::make_shared<oslam::DataProvider>(&m_data_provider_input_queue);
    mp_image_transport =
      std::make_shared<oslam::ImageTransporter>(&m_transport_frame_queue, &m_masked_image_queue);
    mp_tracker = std::make_shared<oslam::Tracker>(&m_masked_image_queue, nullptr);

    //! DataReader fills the DataProvider Input Queue
    auto &data_provider_input_queue = m_data_provider_input_queue;
    mp_data_reader->register_provider_callback(
      [&data_provider_input_queue](
        Frame::UniquePtr p_frame) { data_provider_input_queue.push(std::move(p_frame)); });

    //! DataProvider fills the ImageTransporter Input Queue
    auto &transport_frame_queue = m_transport_frame_queue;
    mp_data_provider->register_output_callback(
      [&transport_frame_queue](const Frame::Ptr &rp_frame) {
          if (rp_frame->is_maskframe()) {
              transport_frame_queue.push(std::make_unique<Frame>(*rp_frame));
          }
      });

    //! DataProvider also fills the Tracker Frame Queue
    mp_data_provider->register_output_callback(
      std::bind(&Tracker::fill_frame_queue, mp_tracker, std::placeholders::_1));

    //! ImageTransporter fills the Masked Image Queue
    /* mp_image_transport->register_output_callback( */
    /*   std::bind(&Tracker::fill_masked_image_queue, mp_tracker, std::placeholders::_1)); */


    /* mp_data_provider->register_output_callback( */
    /* std::bind(&Tracker::fill_frame_queue, mp_tracker, std::placeholders::_1)); */

    //! ImageTransporter returns segmented images back to Tracker
    /* mp_image_transport->register_callback( */
    /*   std::bind(&oslam::Tracker::fill_mask_image_queue, mp_tracker, std::placeholders::_1)); */

    /* mp_tracker->register_callback(std::bind( */
    /*   &Tracker::fill_frame_queue, mp_image_transport, std::placeholders::_1)); */

    /* mp_global_map = std::make_shared<oslam::GlobalMap>(); */


    if (mp_data_reader && mp_image_transport && mp_tracker) return true;
    return false;
}

void Controller::run()
{

    // Start thread for each component
    auto handle_dataset = std::async(std::launch::async, &oslam::DataReader::run, mp_data_reader);

    auto handle_provider =
      std::async(std::launch::async, &oslam::DataProvider::run, mp_data_provider);

    auto handle_image_transport =
      std::async(std::launch::async, &oslam::ImageTransporter::run, mp_image_transport);

    auto handle_tracker = std::async(std::launch::async, &oslam::Tracker::run, mp_tracker);

    /* spdlog::info( */
    /*   "{}, {}", handle_dataset.get(), handle_image_transport.get()); */

    handle_dataset.get();
    handle_provider.get();
    handle_image_transport.get();
}

}// namespace oslam
