/******************************************************************************
 * File:             renderer.cpp
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Implementation file for renderer
 *****************************************************************************/
#include "renderer.h"

namespace oslam
{
    Renderer::Renderer(Map::Ptr map, InputQueue* input_queue, OutputQueue* output_queue)
        : SISO(input_queue, output_queue, "Renderer"), map_(map)
    {
    }

    Renderer::OutputUniquePtr Renderer::runOnce(Renderer::InputUniquePtr input)
    {
        curr_timestamp_                       = input->timestamp_;
        const RendererInput& renderer_payload = *input;
        const Frame& frame                    = renderer_payload.frame_;

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
            background->raycast(
                model_vertices_cuda_, model_normals_cuda_, model_colors_cuda_, renderer_payload.camera_pose_);

            auto raycast_time = Timer::toc(raycast_start).count();
            spdlog::debug("Raycast took {} ms", raycast_time);

            cv::Mat color_map = model_colors_cuda_.DownloadMat();
            cv::Mat vertices  = model_vertices_cuda_.DownloadMat();
            cv::Mat normals   = model_normals_cuda_.DownloadMat();

#ifdef OSLAM_DEBUG_VIS
            /* cv::imshow("Color map", color_map); */
            /* cv::imshow("Source image", frame.color_); */
#endif
            spdlog::debug("Returning render output");
            return std::make_unique<RendererOutput>(curr_timestamp_ + 1, color_map, vertices, normals);
        }
        return nullptr;
    }
}  // namespace oslam
