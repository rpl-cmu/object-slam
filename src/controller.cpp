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
    m_dataset_path(r_args.at("<dataset_path>").asString()), mp_data_reader(nullptr),
    mp_image_transport(nullptr), mp_context(std::make_shared<zmq::context_t>(1))
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
    // TODO(Akash): Read image properties from dataset folder
    /* ImageProperties image_properties({ 480, 640, 3, 1 }); */

    mp_image_transport = std::make_shared<oslam::ImageTransporter>();

    mp_tracker = std::make_shared<oslam::Tracker>(mp_image_transport);

    mp_data_reader->register_callback(
      std::bind(&oslam::Tracker::fill_data_queue, mp_tracker, std::placeholders::_1));

    mp_image_transport->register_callback(
      std::bind(&oslam::Tracker::fill_mask_image_queue, mp_tracker, std::placeholders::_1));

    mp_tracker->register_callback(std::bind(
      &oslam::ImageTransporter::fill_send_frame_queue, mp_image_transport, std::placeholders::_1));

    mp_global_map = std::make_shared<oslam::GlobalMap>();


    if (mp_data_reader && mp_image_transport && mp_tracker) return true;
    return false;
}

void Controller::run()
{

    // Start thread for each component
    // TODO: ImageTransporter does not work!
    /* mvp_threads.push_back(std::thread(&ImageTransporter::start, mp_image_transport)); */
    /* mvp_threads.push_back(std::thread(&Thread::start, mp_tracker)); */

    /* // Join all threads */
    /* for (std::size_t i=0; i < mvp_threads.size(); i++) { */
    /*     if(mvp_threads.at(i).joinable()) */
    /*         mvp_threads.at(i).join(); */
    /* } */
    auto handle_dataset = std::async(std::launch::async, &DataReader::run, mp_data_reader);

    auto handle_tracker = std::async(std::launch::async, &Thread::start, mp_tracker);

    auto handle_image_transport = std::async(std::launch::async, &ImageTransporter::run, mp_image_transport);

    spdlog::info("{}, {}, {}", handle_dataset.get(), handle_tracker.get(), handle_image_transport.get());
}

}// namespace oslam
