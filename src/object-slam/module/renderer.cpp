/******************************************************************************
 * File:             renderer.cpp
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Implementation file for renderer
 *****************************************************************************/
#include "renderer.h"
#include <Eigen/src/Geometry/Hyperplane.h>
#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Open3D.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <fstream>
#include <vector>
#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace oslam
{
    Renderer::Renderer(const Map::Ptr& map, InputQueue* input_queue) : SIMO(input_queue, "Renderer"), map_(map) {}

    Renderer::OutputUniquePtr Renderer::runOnce(Renderer::InputUniquePtr input)
    {
        spdlog::info("Reached renderer runOnce");
        curr_timestamp_                         = input->timestamp_;
        const RendererInput& renderer_payload   = *input;
        const Frame& frame                      = renderer_payload.frame_;
        const Eigen::Matrix3d& intrinsic_matrix = frame.intrinsic_.intrinsic_matrix_ * 4.0;
        const InstanceImage& bg_instance        = renderer_payload.bg_instance_;
        Renders all_renders                     = renderer_payload.object_renders_;

        spdlog::info("Filling camera trajectory");
        std::vector<cv::Affine3d> camera_trajectory_3d = fillCameraTrajectory(map_->getCameraTrajectory());
        spdlog::info("Filled camera_trajectory");

        if (renderer_payload.mapper_status_ == MapperStatus::VALID ||
            renderer_payload.mapper_status_ == MapperStatus::OPTIMIZED)
        {
            auto raycast_start_time = Timer::tic();
            spdlog::info("Mapping status is valid");

            Render background_render = map_->renderBackground(frame, map_->getCameraPose(curr_timestamp_));

            cv::Mat color_map, vertex_map, normal_map;
            background_render.color_map_.copyTo(color_map, bg_instance.maskb_);
            background_render.vertex_map_.copyTo(vertex_map, bg_instance.maskb_);
            background_render.normal_map_.copyTo(normal_map, bg_instance.maskb_);

            cv::imshow("Background render", color_map);

            all_renders.emplace_back(map_->getBackgroundId(), Render(color_map, vertex_map, normal_map));
            /* all_renders.emplace_back(map_->getBackgroundId(), background_render); */

            std::vector<cv::Mat> depth_array;
            std::vector<cv::Mat> color_array;
            std::vector<cv::Mat> vertex_array;
            std::vector<cv::Mat> normal_array;
            depth_array.reserve(all_renders.size());
            color_array.reserve(all_renders.size());
            vertex_array.reserve(all_renders.size());
            normal_array.reserve(all_renders.size());

            for (auto iter = all_renders.begin(); iter != all_renders.end(); ++iter)
            {
                const Render& render = iter->second;
                cv::Mat depth;

                // Extract Z channel as depth of the object vertices
                cv::extractChannel(render.vertex_map_, depth, 2);

                depth_array.push_back(depth);
                color_array.push_back(render.color_map_);
                vertex_array.push_back(render.vertex_map_);
                normal_array.push_back(render.normal_map_);
            }
            // multichannel image with channels = num of objects in camera frustum
            cv::Mat object_depths;
            cv::merge(depth_array, object_depths);

            std::vector<int> shape         = { object_depths.rows, object_depths.cols, object_depths.channels() };
            xt::xarray<float> depth_xarray = xt::adapt((float*)object_depths.data,
                                                       object_depths.total() * static_cast<size_t>(object_depths.channels()),
                                                       xt::no_ownership(),
                                                       shape);

            //! Use a constant greater than max depth threshold
            depth_xarray = xt::where(xt::isfinite(depth_xarray), depth_xarray, 101);
            depth_xarray = xt::where(depth_xarray <= 0, 101, depth_xarray);

            xt::xarray<int> min_idx = xt::argmin(depth_xarray, depth_xarray.dimension() - 1);

            // Just for visualization
            xt::xarray<float> min_depth = xt::amin(depth_xarray, depth_xarray.dimension() - 1);
            cv::Mat min_depth_mat       = cv::Mat(min_depth.shape()[0], min_depth.shape()[1], CV_32FC1, min_depth.data());

            cv::Mat layered_color  = cv::Mat::zeros(frame.color_.rows, frame.color_.cols, CV_8UC3);
            cv::Mat layered_vertex = cv::Mat::zeros(frame.color_.rows, frame.color_.cols, CV_32FC3);
            cv::Mat layered_normal = cv::Mat::zeros(frame.color_.rows, frame.color_.cols, CV_32FC3);

            for (int row = 0; row < frame.color_.rows; ++row)
            {
                for (int col = 0; col < frame.color_.cols; ++col)
                {
                    size_t layer                           = min_idx(row, col);
                    layered_color.at<cv::Vec3b>(row, col)  = color_array.at(layer).at<cv::Vec3b>(row, col);
                    layered_vertex.at<cv::Vec3f>(row, col) = vertex_array.at(layer).at<cv::Vec3f>(row, col);
                    layered_normal.at<cv::Vec3f>(row, col) = normal_array.at(layer).at<cv::Vec3f>(row, col);
                }
            }
            Render::UniquePtr layered_render = std::make_unique<Render>(layered_color, layered_vertex, layered_normal);

            RendererOutput::UniquePtr render_output =
                std::make_unique<RendererOutput>(curr_timestamp_ + 1, std::move(layered_render));

            //! TODO: Mesh output should be part of RendererOutput
            WidgetPtr traj_widget    = render3dTrajectory(camera_trajectory_3d);
            WidgetPtr frustum_widget = render3dFrustumWithColorMap(camera_trajectory_3d, intrinsic_matrix, layered_color);

            auto object_bboxes = map_->getAllObjectBoundingBoxes();
            renderObjectCubes(object_bboxes, render_output->widgets_map_);

            /* if(renderer_payload.mapper_status_ == MapperStatus::OPTIMIZED) */
            /* { */
            /*     auto object_meshes = map_->meshAllObjects(); */
            /*     renderObjectMeshes(object_meshes, render_output->widgets_map_); */
            /* } */
            auto point_planes = map_->getCameraFrustumPlanes(map_->getCameraPose(curr_timestamp_));
            renderFrustumPlanes(point_planes, render_output->widgets_map_);

            auto raycast_finish_time = Timer::toc(raycast_start_time).count();

            spdlog::debug("Raycasting map object took {} ms", raycast_finish_time);

            render_output->widgets_map_.emplace("Trajectory", std::move(traj_widget));
            render_output->widgets_map_.emplace("Frustum", std::move(frustum_widget));
            return render_output;
        }
        return nullptr;
    }

    std::vector<cv::Affine3d> Renderer::fillCameraTrajectory(const PoseTrajectory& camera_trajectory)
    {
        using namespace boost::filesystem;
        std::vector<cv::Affine3d> camera_trajectory_3d;
        camera_trajectory_3d.reserve(camera_trajectory.size());
        path camera_trajectory_file{ current_path() / "camera_trajectory.txt" };
        ofstream trajectory_stream{ camera_trajectory_file };
        for (const auto& camera_pose : camera_trajectory)
        {
            cv::Matx44d cv_camera;
            cv::eigen2cv(camera_pose, cv_camera);
            cv::Affine3d cv_camera_pose(cv_camera);
            trajectory_stream << camera_pose << "\n";
            camera_trajectory_3d.push_back(cv_camera_pose);
        }
        return camera_trajectory_3d;
    }

    WidgetPtr Renderer::render3dTrajectory(const std::vector<cv::Affine3d>& camera_trajectory_3d)
    {
        //! TODO: Limit trajectory length
        return std::make_shared<cv::viz::WTrajectory>(
            camera_trajectory_3d, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
    }

    WidgetPtr Renderer::render3dFrustumTraj(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                            const Eigen::Matrix3d& intrinsic_matrix,
                                            const size_t& num_prev_frustums)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);

        std::vector<cv::Affine3d> trajectory_frustums;
        trajectory_frustums.reserve(num_prev_frustums);
        size_t count = 0;
        for (auto it = camera_trajectory_3d.end(); it != camera_trajectory_3d.begin() && count < num_prev_frustums; --it)
        {
            trajectory_frustums.push_back(*it);
            count++;
        }
        return std::make_unique<cv::viz::WTrajectoryFrustums>(trajectory_frustums, K, 0.2, cv::viz::Color::green());
    }

    WidgetPtr Renderer::render3dFrustumWithColorMap(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                                    const Eigen::Matrix3d& intrinsic_matrix,
                                                    const cv::Mat& color_map)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);
        std::unique_ptr<cv::viz::WCameraPosition> frustum_widget =
            std::make_unique<cv::viz::WCameraPosition>(K, color_map, 0.4, cv::viz::Color::green());
        frustum_widget->setPose(camera_trajectory_3d.back());
        return frustum_widget;
    }

    void Renderer::renderObjectCubes(const ObjectBoundingBoxes& object_bboxes, std::map<std::string, WidgetPtr>& widget_map)
    {
        for (const auto& point_pair : object_bboxes)
        {
            cv::Vec3d min_pt, max_pt;
            cv::eigen2cv(point_pair.second.first, min_pt);
            cv::eigen2cv(point_pair.second.second, max_pt);
            std::string object_id_string = fmt::format("{}", point_pair.first);
            widget_map.emplace(object_id_string, std::make_unique<cv::viz::WCube>(cv::Point3d(min_pt), cv::Point3d(max_pt)));
        }
    }

    void Renderer::renderObjectMeshes(const IdToObjectMesh& object_meshes, std::map<std::string, WidgetPtr>& widget_map)
    {
        for (const auto& mesh_pair : object_meshes)
        {
            cv::Mat colors, vertices, polygons;
            const auto& mesh             = mesh_pair.second;
            std::string object_id_string = fmt::format("{}temp.ply", mesh_pair.first);
            io::WriteTriangleMesh(object_id_string, *mesh);

            auto cv_mesh        = cv::viz::Mesh::load(object_id_string);
            auto cv_mesh_widget = std::make_unique<cv::viz::WMesh>(cv_mesh);
            boost::filesystem::remove(object_id_string);

            widget_map.emplace(object_id_string, std::move(cv_mesh_widget));
        }
    }

    void Renderer::renderFrustumPlanes(const PointPlanes& point_planes, std::map<std::string, WidgetPtr>& widget_map)
    {
        {
            int i = 0;
            for (const auto& point_plane : point_planes)
            {
                const Plane3d& plane                = point_plane.first;
                const Eigen::Vector3d plane_normal  = plane.normal();
                const Eigen::Vector3d& plane_center = point_plane.second;

                cv::Vec3d cv_plane_normal, cv_plane_center;
                cv::eigen2cv(plane_normal, cv_plane_normal);
                cv::eigen2cv(plane_center, cv_plane_center);
                std::string plane_string = fmt::format("plane_{}", i);
                std::string arrow_string = fmt::format("Arrow_{}", i);
                cv::Vec3d second_point = cv_plane_center + 0.25 * cv_plane_normal;
                widget_map.emplace(
                    arrow_string,
                    std::make_unique<cv::viz::WArrow>(cv_plane_center, second_point, 0.02, cv::viz::Color::yellow()));
                /* widget_map.emplace(plane_string, */
                /*                    std::make_unique<cv::viz::WPlane>(cv_plane_center, */
                /*                                                      cv_plane_normal, */
                /*                                                      cv::Vec3d(0, 1, 0), */
                /*                                                      cv::Size2d(0.25, 0.25), */
                /*                                                      cv::viz::Color::red())); */
                i++;
            }
        }
    }
}  // namespace oslam
