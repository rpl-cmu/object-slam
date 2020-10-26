
#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>
#include <Cuda/Open3DCuda.h>
#include <Open3D/Open3D.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

using namespace open3d;
using namespace open3d::registration;
using namespace open3d::geometry;
using namespace open3d::io;
using namespace open3d::utility;
namespace fs = boost::filesystem;

void readFragment(const fs::path& filename) {

    Timer timer;
    timer.Start();
    cuda::ScalableTSDFVolumeCuda tsdf_volume =
            io::ReadScalableTSDFVolumeFromBIN(filename.string(), true);
    std::cout << tsdf_volume.GetMinBound() << "\n";
    std::cout << tsdf_volume.GetMaxBound() << "\n";
    timer.Stop();
    utility::LogInfo("Read takes {} ms\n", timer.GetDuration());
    auto aabb = std::make_shared<AxisAlignedBoundingBox>(
            tsdf_volume.GetMinBound(), tsdf_volume.GetMaxBound());

    tsdf_volume.GetAllSubvolumes();
    cuda::ScalableMeshVolumeCuda mesher(
            cuda::VertexWithNormalAndColor, 16,
            tsdf_volume.active_subvolume_entry_array_.size());
    mesher.MarchingCubes(tsdf_volume);
    auto mesh = mesher.mesh().Download();
    for(auto& color : mesh->vertex_colors_)
    {
        Eigen::Vector3d temp_color = color;
        color(0) = temp_color(2);
        color(1) = temp_color(1);
        color(2) = temp_color(0);
    }
    std::cout << mesh->GetMinBound() << "\n";
    std::cout << mesh->GetMaxBound() << "\n";
    visualization::DrawGeometries({mesh, aabb});
}

int main(int argc, char* argv[]) {
    CLI::App app{ "Read objects BIN file and display mesh" };
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread: %t] %^[%L]  %v%$");
    std::string bin_file;
    app.add_option("bin_file", bin_file, "Path to the bin file")->required();

    CLI11_PARSE(app, argc, argv);

    if(!fs::exists(bin_file) && !fs::is_regular_file(bin_file))
    {
        spdlog::error("Object BIN does not exist");
        return EXIT_FAILURE;
    }
    readFragment(bin_file);
}

