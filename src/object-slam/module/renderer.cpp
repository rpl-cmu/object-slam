/******************************************************************************
 * File:             renderer.cpp
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Implementation file for renderer
 *****************************************************************************/
#include "renderer.h"

#include <opencv2/core/eigen.hpp>

namespace oslam
{
    Renderer::Renderer(const Map::Ptr& map, InputQueue* input_queue) : SIMO(input_queue, "Renderer"), map_(map) {}

    Renderer::OutputUniquePtr Renderer::runOnce(Renderer::InputUniquePtr input)
    {
        curr_timestamp_                       = input->timestamp_;
        const RendererInput& renderer_payload = *input;
        const Frame& frame                    = renderer_payload.frame_;
        const Eigen::Matrix4d& camera_pose    = renderer_payload.camera_pose_;
        const Eigen::Matrix3d& intrinsic_matrix = frame.intrinsic_.intrinsic_matrix_;

        cv::Matx44d cv_camera;
        cv::eigen2cv(camera_pose, cv_camera);
        cv::Affine3d cv_camera_pose(cv_camera);
        camera_trajectory_3d_.push_back(cv_camera_pose);

        if (renderer_payload.mapper_status_ == MapperStatus::VALID)
        {
            auto raycast_start = Timer::tic();

            if (curr_timestamp_ == 1)
            {
                model_colors_cuda_.Create(frame.width_, frame.height_);
                model_vertices_cuda_.Create(frame.width_, frame.height_);
                model_normals_cuda_.Create(frame.width_, frame.height_);
            }

            auto background = map_->getBackground();
            if (!background)
            {
                spdlog::error("Background object not present for rendering");
                return nullptr;
            }
            background->raycast(model_vertices_cuda_, model_normals_cuda_, model_colors_cuda_, camera_pose);

            auto raycast_time = Timer::toc(raycast_start).count();
            spdlog::debug("Raycast took {} ms", raycast_time);

            cv::Mat color_map = model_colors_cuda_.DownloadMat();
            cv::Mat vertices  = model_vertices_cuda_.DownloadMat();
            cv::Mat normals   = model_normals_cuda_.DownloadMat();

            //! TODO: Mesh output should be part of RendererOutput
            WidgetPtr traj_widget    = render3dTrajectory();
            WidgetPtr frustum_widget = render3dFrustumWithColorMap(intrinsic_matrix, color_map);

            RendererOutput::UniquePtr render_output =
                std::make_unique<RendererOutput>(curr_timestamp_ + 1, color_map, vertices, normals);

            render_output->widgets_map_.emplace("Trajectory", std::move(traj_widget));
            render_output->widgets_map_.emplace("Frustum", std::move(frustum_widget));
            return render_output;
        }
        return nullptr;
    }

    WidgetPtr Renderer::render3dTrajectory()
    {
        //! TODO: Limit trajectory length
        return std::make_shared<cv::viz::WTrajectory>(camera_trajectory_3d_, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
    }

    WidgetPtr Renderer::render3dFrustumTraj(const Eigen::Matrix3d& intrinsic_matrix, const size_t& num_prev_frustums)
    {
        cv::Matx33d K;
        cv::eigen2cv(intrinsic_matrix, K);

        std::vector<cv::Affine3d> trajectory_frustums;
        trajectory_frustums.reserve(num_prev_frustums);
        size_t count = 0;
        for(auto it = camera_trajectory_3d_.end(); it != camera_trajectory_3d_.begin() && count < num_prev_frustums; --it)
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
