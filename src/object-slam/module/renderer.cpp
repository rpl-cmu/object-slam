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
    Renderer::Renderer(const Map::Ptr& map, InputQueue* input_queue) : SIMO(input_queue, "Renderer"), map_(map)
    {
        spdlog::trace("CONSTRUCT: Renderer");
    }

    Renderer::OutputUniquePtr Renderer::runOnce(Renderer::InputUniquePtr input)
    {
        spdlog::trace("Renderer::runOnce()");
        curr_timestamp_                         = input->timestamp_;
        const RendererInput& renderer_payload   = *input;
        const Frame& frame                      = renderer_payload.frame_;
        const Eigen::Matrix3d& intrinsic_matrix = frame.intrinsic_.intrinsic_matrix_ * 4.0;
        const InstanceImage& bg_instance        = renderer_payload.bg_instance_;

        std::vector<cv::Affine3d> camera_trajectory_3d = fillCameraTrajectory(map_->getCameraTrajectory());

        if (renderer_payload.mapper_status_ == MapperStatus::VALID ||
            renderer_payload.mapper_status_ == MapperStatus::OPTIMIZED)
        {
            auto render_start_time = Timer::tic();

            Render background_render = map_->renderBackground(frame, map_->getCameraPose(curr_timestamp_));
            Renders all_renders      = map_->renderObjects(frame, map_->getCameraPose(curr_timestamp_));
            spdlog::info("Rendering objects took {} ms", Timer::toc(render_start_time).count());

            cv::Mat color_map, vertex_map, normal_map;
            background_render.color_map_.copyTo(color_map, bg_instance.maskb_);
            background_render.vertex_map_.copyTo(vertex_map, bg_instance.maskb_);
            background_render.normal_map_.copyTo(normal_map, bg_instance.maskb_);

            cv::imshow("Background render", color_map);

            all_renders.emplace_back(map_->getBackgroundId(), Render(color_map, vertex_map, normal_map));

            Render model_render = background_render;
            //! Use compositional render only after 25 frames
            if(camera_trajectory_3d.size() > 25)
                model_render = composeSceneRenders(all_renders);

            all_renders.pop_back();
            all_renders.emplace_back(map_->getBackgroundId(), background_render);

            RendererOutput::UniquePtr render_output =
                std::make_unique<RendererOutput>(curr_timestamp_ + 1, model_render, all_renders);

            auto object_bboxes = map_->getAllObjectBoundingBoxes();
            auto point_planes  = map_->getCameraFrustumPlanes(map_->getCameraPose(curr_timestamp_));

            render_output->widgets_map_.emplace("Trajectory", render3dTrajectory(camera_trajectory_3d));
            render_output->widgets_map_.emplace(
                "Frustum", render3dFrustumWithColorMap(camera_trajectory_3d, intrinsic_matrix, model_render.color_map_));
            renderObjectCubes(object_bboxes, render_output->widgets_map_);
            renderFrustumPlanes(point_planes, render_output->widgets_map_);

            /* if(renderer_payload.mapper_status_ == MapperStatus::OPTIMIZED) */
            /* { */
            /*     auto object_meshes = map_->meshAllObjects(); */
            /*     renderObjectMeshes(object_meshes, render_output->widgets_map_); */
            /* } */

            spdlog::info("Renderer took {} ms", Timer::toc(render_start_time).count());
            return render_output;
        }
        return nullptr;
    }

    Render Renderer::composeSceneRenders(const Renders& renders) const
    {
        std::vector<cv::Mat> depth_array, color_array, vertex_array, normal_array;

        for (auto iter = renders.begin(); iter != renders.end(); ++iter)
        {
            const Render& render = iter->second;

            cv::Mat depth;
            cv::extractChannel(render.vertex_map_, depth, 2);
            depth_array.push_back(depth);

            color_array.push_back(render.color_map_);
            vertex_array.push_back(render.vertex_map_);
            normal_array.push_back(render.normal_map_);
        }
        cv::Mat object_depths;
        cv::merge(depth_array, object_depths);

        //! TODO: Using xtensor only for this one task is not warranted.
        std::vector<int> shape         = { object_depths.rows, object_depths.cols, object_depths.channels() };
        xt::xarray<float> depth_xarray = xt::adapt((float*)object_depths.data,
                                                   object_depths.total() * static_cast<size_t>(object_depths.channels()),
                                                   xt::no_ownership(),
                                                   shape);

        //! Use a constant greater than max depth threshold
        depth_xarray = xt::where(xt::isfinite(depth_xarray), depth_xarray, 100);
        depth_xarray = xt::where(depth_xarray <= 0, 100, depth_xarray);

        xt::xarray<size_t> min_idx = xt::argmin(depth_xarray, depth_xarray.dimension() - 1);

        cv::Mat layered_color  = cv::Mat::zeros(object_depths.rows, object_depths.cols, CV_8UC3);
        cv::Mat layered_vertex = cv::Mat::zeros(object_depths.rows, object_depths.cols, CV_32FC3);
        cv::Mat layered_normal = cv::Mat::zeros(object_depths.rows, object_depths.cols, CV_32FC3);

        for (int row = 0; row < object_depths.rows; ++row)
        {
            for (int col = 0; col < object_depths.cols; ++col)
            {
                size_t layer                           = min_idx(row, col);
                layered_color.at<cv::Vec3b>(row, col)  = color_array.at(layer).at<cv::Vec3b>(row, col);
                layered_vertex.at<cv::Vec3f>(row, col) = vertex_array.at(layer).at<cv::Vec3f>(row, col);
                layered_normal.at<cv::Vec3f>(row, col) = normal_array.at(layer).at<cv::Vec3f>(row, col);
            }
        }
        return Render(layered_color, layered_vertex, layered_normal);
    }

    std::vector<cv::Affine3d> Renderer::fillCameraTrajectory(const PoseTrajectory& camera_trajectory)
    {
        std::vector<cv::Affine3d> camera_trajectory_3d;
        camera_trajectory_3d.reserve(camera_trajectory.size());
        for (const auto& camera_pose : camera_trajectory)
        {
            cv::Matx44d cv_camera;
            cv::eigen2cv(camera_pose, cv_camera);
            cv::Affine3d cv_camera_pose(cv_camera);
            camera_trajectory_3d.push_back(cv_camera_pose);
        }
        return camera_trajectory_3d;
    }

    WidgetPtr Renderer::render3dTrajectory(const std::vector<cv::Affine3d>& camera_trajectory_3d) const
    {
        //! TODO: Limit trajectory length
        return std::make_shared<cv::viz::WTrajectory>(
            camera_trajectory_3d, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::green());
    }

    WidgetPtr Renderer::render3dFrustumTraj(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                            const Eigen::Matrix3d& intrinsic_matrix,
                                            const size_t& num_prev_frustums) const
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
        return std::make_unique<cv::viz::WTrajectoryFrustums>(trajectory_frustums, K, 0.2, cv::viz::Color::red());
    }

    WidgetPtr Renderer::render3dFrustumWithColorMap(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                                    const Eigen::Matrix3d& intrinsic_matrix,
                                                    const cv::Mat& color_map) const
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);
        std::unique_ptr<cv::viz::WCameraPosition> frustum_widget =
            std::make_unique<cv::viz::WCameraPosition>(K, color_map, 0.4, cv::viz::Color::red());
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
                cv::Vec3d second_point   = cv_plane_center + 0.25 * cv_plane_normal;
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
