#include <Open3D/Geometry/RGBDImage.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <iostream>
#include <memory>
#include <thread>

#include <docopt/docopt.h>
#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <Open3D/Open3D.h>

#include "dataset.h"

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

    spdlog::set_level(spdlog::level::debug);

    std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
      { std::next(argv), std::next(argv, argc) },
      true,// show help if requested
      "Object SLAM 0.1");// version string

    const auto dataset_path = args["<dataset_path>"].asString();

    fmt::print("{}\n", dataset_path);

    oslam::RGBDdataset rgbd_dataset = oslam::RGBDdataset(dataset_path);

    for (std::size_t i = 0; i < rgbd_dataset.size(); i++) {
        oslam::RGBDdata data = rgbd_dataset.get_data(0);
        auto rgbd_image =
          open3d::geometry::RGBDImage::CreateFromRedwoodFormat(data.color, data.depth, false);
        auto pcd =
          open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_image, rgbd_dataset.intrinsic);
        open3d::visualization::DrawGeometries({ pcd }, "PointCloud visualization");
    }
}
