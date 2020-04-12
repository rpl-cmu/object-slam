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

#include <Open3D/Open3D.h>
#include <utility>

#include "dataset.h"
#include "frame.h"
#include "image_transport.h"

static constexpr auto USAGE =
  R"(Object SLAM Pipeline

    Usage:
          reconstruct <dataset_path>
          reconstruct (-h | --help)

    Options:
          -h --help    Show the help screen
    )";

int main(int argc, char *argv[])
{

    spdlog::set_level(spdlog::level::info);

    std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
      { std::next(argv), std::next(argv, argc) },
      true,// show help if requested
      "Object SLAM 0.1");// version string

    const auto dataset_path = args["<dataset_path>"].asString();


    oslam::RGBDdataset rgbd_dataset = oslam::RGBDdataset(dataset_path);
    oslam::ImageTransporter image_transport = oslam::ImageTransporter({480, 640, 3, 1});

    std::shared_ptr<oslam::Frame> p_prev_frame;
    std::unique_ptr<oslam::MaskedImage> p_masked_image;
    for (std::size_t i = 0; i < rgbd_dataset.size(); i++) {
        image_transport.send(rgbd_dataset.get_data(i).color);
        std::shared_ptr<oslam::Frame> p_current_frame(new oslam::Frame(i, rgbd_dataset.get_data(i), rgbd_dataset.intrinsic));

        std::shared_ptr<oslam::Odometry> p_odometry = p_current_frame->odometry(p_prev_frame);

        if(image_transport.p_masked_image)
        {
            p_masked_image = std::move(image_transport.p_masked_image);
            unsigned int l = 0;
            for (auto label : p_masked_image->labels)
            {
                spdlog::debug("Label {}: {}", l++, label);
            }
        }
        if(p_prev_frame)
            p_current_frame->set_pose(p_prev_frame->get_pose() * p_odometry->transform);
        spdlog::info("Frame current pose: \n{}\n", p_current_frame->get_pose());
        p_current_frame->visualize();

        p_prev_frame = p_current_frame;
    }
}
