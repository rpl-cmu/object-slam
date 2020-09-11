/******************************************************************************
 * File:             renderer.cpp
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Implementation file for renderer
 *****************************************************************************/
#include "renderer.h"

#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
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
        const Eigen::Matrix4d& camera_pose           = renderer_payload.camera_pose_;
        const Eigen::Matrix3d& intrinsic_matrix      = frame.intrinsic_.intrinsic_matrix_;
        const ObjectRendersUniquePtr& object_renders = renderer_payload.object_renders_;

        cv::Matx44d cv_camera;
        cv::eigen2cv(camera_pose, cv_camera);
        cv::Affine3d cv_camera_pose(cv_camera);
        camera_trajectory_3d_.push_back(cv_camera_pose);

        if (renderer_payload.mapper_status_ == MapperStatus::VALID)
        {
            auto raycast_start_time = Timer::tic();

            if (object_renders)
            {
                ObjectRender::UniquePtr background_render = nullptr;
                for (auto iter = map_->id_to_object_.begin(); iter != map_->id_to_object_.end(); ++iter)
                {
                    const ObjectId& id            = iter->first;
                    const TSDFObject::Ptr& object = iter->second;
                    if (object->isBackground())
                    {
                        const ObjectRender& object_render = object_renders->at(id);
                        background_render                 = std::make_unique<ObjectRender>(
                            object_render.color_map_, object_render.vertex_map_, object_render.normal_map_);
                    }
                }

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

                std::vector<int> shape         = { object_depths.cols, object_depths.rows, object_depths.channels() };
                xt::xarray<float> depth_xarray = xt::adapt(
                    (float*)object_depths.data, object_depths.total() * static_cast<size_t>(object_depths.channels()), xt::no_ownership(), shape);

                spdlog::info("Object depth array shape: {}", depth_xarray.shape());
                xt::xarray<int> min_idx = xt::argmin(depth_xarray, depth_xarray.dimension() - 1);

                xt::xarray<float> min_depth = xt::amin(depth_xarray, depth_xarray.dimension() - 1);

                std::vector<int> shape_min(min_depth.shape().begin(), min_depth.shape().end());
                const int* shape_min_ptr = shape.data();
                cv::Mat min_depth_mat = cv::Mat(min_depth.shape()[1], min_depth.shape()[0], CV_32FC1, min_depth.data());

                cv::imshow("Min depth matrix", min_depth_mat);

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

                cv::imshow("Layered color", layered_color);
                cv::imshow("Layered normal", layered_vertex);

                //! TODO: Mesh output should be part of RendererOutput
                WidgetPtr traj_widget    = render3dTrajectory();
                WidgetPtr frustum_widget = render3dFrustumWithColorMap(intrinsic_matrix, background_render->color_map_);

                RendererOutput::UniquePtr render_output =
                    std::make_unique<RendererOutput>(curr_timestamp_ + 1, std::move(background_render));

                auto raycast_finish_time = Timer::toc(raycast_start_time).count();

                spdlog::debug("Raycasting map object took {} ms", raycast_finish_time);

                render_output->widgets_map_.emplace("Trajectory", std::move(traj_widget));
                render_output->widgets_map_.emplace("Frustum", std::move(frustum_widget));
                return render_output;
            }
        }
        return nullptr;
    }

    WidgetPtr Renderer::render3dTrajectory()
    {
        //! TODO: Limit trajectory length
        return std::make_shared<cv::viz::WTrajectory>(
            camera_trajectory_3d_, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
    }

    WidgetPtr Renderer::render3dFrustumTraj(const Eigen::Matrix3d& intrinsic_matrix, const size_t& num_prev_frustums)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);

        std::vector<cv::Affine3d> trajectory_frustums;
        trajectory_frustums.reserve(num_prev_frustums);
        size_t count = 0;
        for (auto it = camera_trajectory_3d_.end(); it != camera_trajectory_3d_.begin() && count < num_prev_frustums; --it)
        {
            trajectory_frustums.push_back(*it);
            count++;
        }
        return std::make_unique<cv::viz::WTrajectoryFrustums>(trajectory_frustums, K, 0.2, cv::viz::Color::green());
    }

    WidgetPtr Renderer::render3dFrustumWithColorMap(const Eigen::Matrix3d& intrinsic_matrix, const cv::Mat& color_map)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);
        std::unique_ptr<cv::viz::WCameraPosition> frustum_widget =
            std::make_unique<cv::viz::WCameraPosition>(K, color_map, 1.0, cv::viz::Color::green());
        frustum_widget->setPose(camera_trajectory_3d_.back());
        return frustum_widget;
    }
}  // namespace oslam
