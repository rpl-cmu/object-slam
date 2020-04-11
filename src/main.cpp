/******************************************************************************
* File:             main.cpp
*
* Author:           Akash Sharma
* Created:          04/07/20
* Description:      main implementation
*****************************************************************************/
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include <docopt/docopt.h>
#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <zmq.hpp>

#include <Open3D/Open3D.h>

#include "dataset.h"
#include "frame.h"

static constexpr auto USAGE =
  R"(Object SLAM Pipeline

    Usage:
          reconstruct <dataset_path>
          reconstruct (-h | --help)

    Options:
          -h --help    Show the help screen
    )";

static constexpr auto PUBLISH_ENDPOINT = "tcp://*:4242";
static constexpr auto SUBSCRIBE_ENDPOINT = "tcp://*.4243";


int main(int argc, char *argv[])
{

    spdlog::set_level(spdlog::level::debug);

    std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
      { std::next(argv), std::next(argv, argc) },
      true,// show help if requested
      "Object SLAM 0.1");// version string

    const auto dataset_path = args["<dataset_path>"].asString();

    zmq::context_t context;
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(PUBLISH_ENDPOINT);

    oslam::RGBDdataset rgbd_dataset = oslam::RGBDdataset(dataset_path);

    std::shared_ptr<oslam::Frame> p_prev_frame;
    for (std::size_t i = 0; i < rgbd_dataset.size(); i++) {
        auto curr_frame_data = rgbd_dataset.get_data(i);
        auto color_image = curr_frame_data.color;
        auto buf = zmq::buffer(color_image.data_);
        publisher.send(buf, zmq::send_flags::dontwait);

        std::shared_ptr<oslam::Frame> p_current_frame(new oslam::Frame(i, rgbd_dataset.get_data(i), rgbd_dataset.intrinsic));

        std::shared_ptr<oslam::Odometry> p_odometry = p_current_frame->odometry(p_prev_frame);

        if(p_prev_frame)
            p_current_frame->set_pose(p_prev_frame->get_pose() * p_odometry->transform);
        spdlog::debug("Frame current pose: \n{}\n", p_current_frame->get_pose());
        p_current_frame->visualize();

        p_prev_frame = p_current_frame;
    }
}
