/******************************************************************************
 * File:             controller.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#include "controller.h"
#include <memory>

namespace oslam {

Controller::Controller(const std::map<std::string, docopt::value> &r_args)
  : m_dataset_path(r_args.at("<dataset_path>").asString()), m_visualize(r_args.at("--vis")),
    m_debug(r_args.at("--debug"))
{
    spdlog::info("m_debug, m_visualize: {}, {}", m_debug, m_visualize);
    if (m_debug) spdlog::set_level(spdlog::level::debug);

    mp_rgbd_dataset = std::make_unique<oslam::RGBDdataset>(m_dataset_path);
    ImageProperties image_properties({ 480, 640, 3, 1 });
    mp_image_transport = std::make_unique<oslam::ImageTransporter>(image_properties);

    spdlog::info("Initialized Object - SLAM with tid: {}", std::this_thread::get_id());
}

void Controller::run()
{
    std::shared_ptr<oslam::Frame> p_prev_frame;

    for (std::size_t i = 0; i < mp_rgbd_dataset->size(); i++) {
        mp_image_transport->send(mp_rgbd_dataset->get_data(i).color);
        std::shared_ptr<oslam::Frame> p_current_frame(
          new oslam::Frame(i, mp_rgbd_dataset->get_data(i), mp_rgbd_dataset->intrinsic));

        std::shared_ptr<oslam::Odometry> p_odometry = p_current_frame->odometry(p_prev_frame);

        // Some frames will have object_rgbd field populated
        if (mp_image_transport->p_masked_image) {
            p_current_frame->process_mask(std::move(mp_image_transport->p_masked_image));
        }
        if (p_prev_frame)
            p_current_frame->set_pose(p_prev_frame->get_pose() * p_odometry->transform);
        spdlog::info("Frame current pose: \n{}\n", p_current_frame->get_pose());

        if (m_visualize) p_current_frame->visualize();

        p_prev_frame = p_current_frame;
    }
}

}// namespace oslam
