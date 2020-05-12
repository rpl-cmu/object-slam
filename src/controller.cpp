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

namespace oslam {

Controller::Controller(const std::map<std::string, docopt::value> &r_args)
  : m_visualize(r_args.at("--vis")), m_debug(r_args.at("--debug")),
    m_dataset_path(r_args.at("<dataset_path>").asString()),
    mp_rgbd_dataset(nullptr),
    mp_image_transport(nullptr),
    mp_context(std::make_shared<zmq::context_t>(1))
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
    if (setup()) {
        run();
        return 0;
    }
    return -1;
}

bool Controller::setup()
{
    mp_rgbd_dataset = std::make_shared<oslam::RGBDdataset>(m_dataset_path);
    ImageProperties image_properties({ 480, 640, 3, 1 });
    mp_image_transport = std::make_shared<oslam::ImageTransporter>(image_properties, mp_context);

    mp_tracker = std::make_shared<oslam::Tracker>(mp_rgbd_dataset, mp_image_transport);

    if(mp_rgbd_dataset && mp_image_transport && mp_tracker)
        return true;
    return false;
}

void Controller::run()
{

    // Start thread for each component
    // TODO: ImageTransporter does not work!
    /* mvp_threads.push_back(std::thread(&ImageTransporter::start, mp_image_transport)); */
    mvp_threads.push_back(std::thread(&Thread::start, mp_tracker));

    // Join all threads
    for (std::size_t i=0; i < mvp_threads.size(); i++) {
        if(mvp_threads.at(i).joinable())
            mvp_threads.at(i).join();
    }

}

}// namespace oslam
