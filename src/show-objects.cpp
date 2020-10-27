
#include <CLI/CLI.hpp>
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <spdlog/spdlog.h>
#include <Cuda/Open3DCuda.h>
#include <Open3D/Open3D.h>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

using namespace open3d;
using namespace open3d::registration;
using namespace open3d::geometry;
using namespace open3d::io;
using namespace open3d::utility;
namespace fs = boost::filesystem;

std::shared_ptr<LineSet> visualizeCameraTrajectory(const std::vector<Eigen::Matrix4d>& camera_trajectory) {
    std::shared_ptr<LineSet> cam_traj_vis =
        std::make_shared<LineSet>();

    int cnt = 0;

    const int kPointsPerFrustum = 5;
    const int kEdgesPerFrustum = 8;

    std::vector<Eigen::Matrix4d> subsampled_cam_traj;
    for(size_t i = 0; i < camera_trajectory.size(); i++)
    {
        if(i %10 == 0)
            subsampled_cam_traj.push_back(camera_trajectory.at(i));
    }
    Eigen::Matrix4d previous_camera_pose = Eigen::Matrix4d::Identity();
    int i = 0;
    for (auto& pose : subsampled_cam_traj) {

        double norm = 0.1;
        Eigen::Vector4d ph;

        ph = pose * Eigen::Vector4d(0, 0, 0, 1);
        cam_traj_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(1, 1, 2, 1/norm));
        cam_traj_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(1, -1, 2, 1/norm));
        cam_traj_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(-1, -1, 2, 1/norm));
        cam_traj_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(-1, 1, 2, 1/norm));
        cam_traj_vis->points_.emplace_back(ph.hnormalized());

        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 1));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 2));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 3));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 4));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 1, cnt + 2));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 2, cnt + 3));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 3, cnt + 4));
        cam_traj_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 4, cnt + 1));

        for (int k = 0; k < kEdgesPerFrustum; ++k) {
            cam_traj_vis->colors_.emplace_back(Eigen::Vector3d(1, 0, 0));
        }


        if(i != 0)
        {
            cam_traj_vis->lines_.emplace_back(
                    Eigen::Vector2i((i-1) * kPointsPerFrustum, i * kPointsPerFrustum));
                cam_traj_vis->colors_.emplace_back(Eigen::Vector3d(0, 0, 1));
        }
        cnt += kPointsPerFrustum;
        i++;
    }

    /* for (auto &edge : pose_graph.edges_) { */
    /*     int s = edge.source_node_id_; */
    /*     int t = edge.target_node_id_; */

    /*     if (edge.uncertain_) { */
    /*         cam_traj_vis->lines_.emplace_back( */
    /*             Eigen::Vector2i(s * kPointsPerFrustum, t * kPointsPerFrustum)); */
    /*         cam_traj_vis->colors_.emplace_back(Eigen::Vector3d(0, 1, 0)); */
    /*     } else { */
    /*         cam_traj_vis->lines_.emplace_back( */
    /*             Eigen::Vector2i(s * kPointsPerFrustum, t * kPointsPerFrustum)); */
    /*         cam_traj_vis->colors_.emplace_back(Eigen::Vector3d(0, 0, 1)); */
    /*     } */
    /* } */

    return cam_traj_vis;
}

std::vector<Eigen::Matrix4d> readCameraTrajectory(const fs::path& filename)
{
    fs::ifstream cam_traj_file{filename};
    spdlog::info("Read file");
    std::string line;
    Eigen::Matrix4d matrix4d = Eigen::Matrix4d::Zero();
    std::vector<Eigen::Matrix4d> camera_trajectory;
    for(int count = 0; std::getline(cam_traj_file, line);)
    {
        std::stringstream ss(line);
        spdlog::info("Processed line: {}", line);
        ss >> matrix4d(count + 0*4) >> matrix4d(count + 1*4) >> matrix4d(count + 2*4) >> matrix4d(count + 3*4);

        count = (count + 1) % 4;
        if(count % 4 == 0)
        {
            camera_trajectory.push_back(matrix4d);
            spdlog::info("Camera pose: {}", matrix4d);
            matrix4d = Eigen::Matrix4d::Zero();

        }
    }
    return camera_trajectory;
}
std::shared_ptr<geometry::TriangleMesh> readFragment(const fs::path& filename) {

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
    /* std::cout << mesh->GetMinBound() << "\n"; */
    /* std::cout << mesh->GetMaxBound() << "\n"; */
    return mesh;
    /* visualization::DrawGeometries({mesh, aabb}); */
}

std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> visualizeObjectMeshes(fs::path object_path)
{
    std::vector<std::shared_ptr<geometry::TriangleMesh>> objects;
    for (auto &file : fs::directory_iterator(object_path))
    {
        if(fs::is_regular_file(file))
        {
            objects.push_back(readFragment(file));
        }
    }
    return objects;
}

int main(int argc, char* argv[]) {
    CLI::App app{ "Read objects BIN file and display mesh" };
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread: %t] %^[%L]  %v%$");
    std::string output_folder;
    app.add_option("outout_folder", output_folder, "Path to the output folder")->required();

    CLI11_PARSE(app, argc, argv);
    fs::path output_path{output_folder};

    if(!fs::exists(output_path) && !fs::is_directory(output_path))
    {
        spdlog::error("Output folder does not exist");
        return EXIT_FAILURE;
    }

    fs::path camera_trajectory_file = output_path / "camera_trajectory.txt";
    if(!fs::exists(camera_trajectory_file) && !fs::is_regular_file(camera_trajectory_file))
    {
        spdlog::error("Camera trajectory file does not exist");
        return EXIT_FAILURE;
    }

    fs::path object_path = output_path / "objects";
    if(!fs::exists(object_path) && !fs::is_directory(object_path))
    {
        spdlog::error("Objects folder absent");
        return EXIT_FAILURE;
    }

    auto camera_trajectory = readCameraTrajectory(camera_trajectory_file);
    std::shared_ptr<open3d::geometry::LineSet> camera_traj_vis = visualizeCameraTrajectory(camera_trajectory);

    auto mesh_vis = visualizeObjectMeshes(object_path);
    std::vector<std::shared_ptr<const geometry::Geometry>> vis{};
    vis.push_back(camera_traj_vis);
    vis.insert(vis.end(), mesh_vis.begin(), mesh_vis.end());
    visualization::DrawGeometries(vis);
    /* readFragment(bin_file); */
}

