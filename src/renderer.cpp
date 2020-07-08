/******************************************************************************
* File:             renderer.cpp
*
* Author:           Akash Sharma
* Created:          07/07/20
* Description:      Implementation file for renderer
*****************************************************************************/
#include "renderer.h"

namespace oslam {

    Renderer::Renderer(InputQueue* p_input_queue, OutputQueue* p_output_queue)
        : SISO(p_input_queue, p_output_queue, "Renderer"),
        mr_global_map(GlobalMap::get_instance())
    {}

    Renderer::OutputUniquePtr Renderer::run_once(Renderer::InputUniquePtr p_input)
    {
        m_curr_timestamp = p_input->m_timestamp;
        RendererInput& renderer_payload = *p_input;
        const Frame& r_curr_frame = renderer_payload.m_frame;

        if(renderer_payload.m_mapper_status == MapperStatus::VALID)
        {
            auto raycast_start = Timer::tic();

            if(m_curr_timestamp == 1)
            {
                mc_current_global_colors.Create(r_curr_frame.m_width, r_curr_frame.m_height);
                mc_current_global_vertices.Create(r_curr_frame.m_width, r_curr_frame.m_height);
                mc_current_global_normals.Create(r_curr_frame.m_width, r_curr_frame.m_height);
            }

            mr_global_map.raycast_background(mc_current_global_vertices, mc_current_global_normals,
                                             mc_current_global_colors, renderer_payload.m_camera_pose);

            auto raycast_time = Timer::toc(raycast_start).count();
            spdlog::debug("Raycast took {} ms", raycast_time);

            cv::Mat color_map = mc_current_global_colors.DownloadMat();
            cv::Mat vertices = mc_current_global_vertices.DownloadMat();
            cv::Mat normals = mc_current_global_normals.DownloadMat();

            /* cv::imshow("Color map", color_map); */
            /* cv::imshow("Source image", r_curr_frame.m_color); */
            /* cv::waitKey(1); */

            return std::make_unique<RendererOutput>(m_curr_timestamp+1, color_map, vertices, normals);
        }
        return nullptr;
    }
}

