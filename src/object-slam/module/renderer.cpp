/******************************************************************************
 * File:             renderer.cpp
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Implementation file for renderer
 *****************************************************************************/
#include "renderer.h"

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
        curr_timestamp_                              = input->timestamp_;
        const RendererInput& renderer_payload        = *input;
        const Frame& frame                           = renderer_payload.frame_;
        const PoseTrajectory& camera_trajectory      = renderer_payload.camera_trajectory_;
        const Eigen::Matrix3d& intrinsic_matrix      = frame.intrinsic_.intrinsic_matrix_;
        const ObjectRendersUniquePtr& object_renders = renderer_payload.object_renders_;

        std::vector<cv::Affine3d> camera_trajectory_3d = fillCameraTrajectory(camera_trajectory);
        spdlog::info("Filled camera_trajectory");

        if (renderer_payload.mapper_status_ == MapperStatus::VALID)
        {
            auto raycast_start_time = Timer::tic();
            spdlog::info("Mapping status is valid");
            if (object_renders)
            {
                // Get background render (Temporary)
                /* ObjectRender::UniquePtr background_render = nullptr; */
                /* for (auto iter = map_->id_to_object_.begin(); iter != map_->id_to_object_.end(); ++iter) */
                /* { */
                /*     const ObjectId& id            = iter->first; */
                /*     const TSDFObject::Ptr& object = iter->second; */
                /*     if (object->isBackground()) */
                /*     { */
                /*         const ObjectRender& object_render = object_renders->at(id); */
                /*         background_render                 = std::make_unique<ObjectRender>( */
                /*             object_render.color_map_, object_render.vertex_map_, object_render.normal_map_); */
                /*     } */
                /* } */
                /* spdlog::info("Obtained background render"); */

                std::vector<cv::Mat> depth_array;
                std::vector<cv::Mat> color_array;
                std::vector<cv::Mat> vertex_array;
                std::vector<cv::Mat> normal_array;
                depth_array.reserve(object_renders->size());
                color_array.reserve(object_renders->size());
                vertex_array.reserve(object_renders->size());
                normal_array.reserve(object_renders->size());

                for (auto iter = object_renders->begin(); iter != object_renders->end(); ++iter)
                {
                    const ObjectRender& object_render = iter->second;
                    cv::Mat depth;
                    // Extract Z channel as depth of the object vertices
                    cv::extractChannel(object_render.vertex_map_, depth, 2);
                    depth_array.push_back(depth);
                    color_array.push_back(object_render.color_map_);
                    vertex_array.push_back(object_render.vertex_map_);
                    normal_array.push_back(object_render.normal_map_);
                }
                // multichannel image with channels = num of objects in camera frustum
                cv::Mat object_depths;
                cv::merge(depth_array, object_depths);

                std::vector<int> shape = { object_depths.rows, object_depths.cols, object_depths.channels() };
                xt::xarray<float> depth_xarray =
                    xt::adapt((float*)object_depths.data,
                              object_depths.total() * static_cast<size_t>(object_depths.channels()),
                              xt::no_ownership(),
                              shape);

                //! Use a constant greater than max depth threshold
                depth_xarray = xt::where(xt::isfinite(depth_xarray), depth_xarray, 101);

                xt::xarray<int> min_idx = xt::argmin(depth_xarray, depth_xarray.dimension() - 1);

                // Just for visualization
                xt::xarray<float> min_depth = xt::amin(depth_xarray, depth_xarray.dimension() - 1);
                cv::Mat min_depth_mat = cv::Mat(min_depth.shape()[0], min_depth.shape()[1], CV_32FC1, min_depth.data());

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
                /* cv::imshow("Layered color image", layered_color); */
                ObjectRender::UniquePtr layered_render = std::make_unique<ObjectRender>(layered_color, layered_vertex,layered_normal);

                //! TODO: Mesh output should be part of RendererOutput
                WidgetPtr traj_widget    = render3dTrajectory(camera_trajectory_3d);
                WidgetPtr frustum_widget = render3dFrustumWithColorMap(camera_trajectory_3d, intrinsic_matrix, layered_render->color_map_);

                RendererOutput::UniquePtr render_output =
                    std::make_unique<RendererOutput>(curr_timestamp_ + 1, std::move(layered_render));

                auto raycast_finish_time = Timer::toc(raycast_start_time).count();

                spdlog::debug("Raycasting map object took {} ms", raycast_finish_time);

                render_output->widgets_map_.emplace("Trajectory", std::move(traj_widget));
                render_output->widgets_map_.emplace("Frustum", std::move(frustum_widget));
                return render_output;
            }
        }
        return nullptr;
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

    WidgetPtr Renderer::render3dFrustumWithColorMap(const std::vector<cv::Affine3d>& camera_trajectory_3d, const Eigen::Matrix3d& intrinsic_matrix, const cv::Mat& color_map)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);
        std::unique_ptr<cv::viz::WCameraPosition> frustum_widget =
            std::make_unique<cv::viz::WCameraPosition>(K, color_map, 1.0, cv::viz::Color::green());
        frustum_widget->setPose(camera_trajectory_3d.back());
        return frustum_widget;
    }
}  // namespace oslam
