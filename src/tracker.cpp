/******************************************************************************
 * File:             tracker.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker thread
 *****************************************************************************/
#include "tracker.h"

#include <Cuda/Common/UtilsCuda.h>
#include <opencv2/opencv.hpp>
#include <memory>

namespace oslam {

Tracker::Tracker(MaskedImageQueue* p_masked_image_queue, OutputQueue* p_output_queue)
  : MISO(p_output_queue, "Tracker"),
    m_frame_queue("InputFrameQueue"), mp_masked_image_queue(p_masked_image_queue)
{}

Tracker::InputUniquePtr Tracker::get_input_packet(void)
{
    auto start_time = Timer::tic();

    Frame::UniquePtr p_input_frame;
    bool queue_state = false;
    queue_state = m_frame_queue.popBlocking(p_input_frame);

    if(!queue_state)
    {
        spdlog::error("Module: {} {} is down", name_id_, mp_masked_image_queue->queue_id_);
        return nullptr;
    }
    if(!p_input_frame)
        spdlog::error("Module: {} {} returned null", name_id_, mp_masked_image_queue->queue_id_);

    m_curr_timestamp = p_input_frame->m_timestamp;
    const bool is_current_maskframe = p_input_frame->is_maskframe();

    Tracker::InputUniquePtr p_tracker_payload;
    if(!is_current_maskframe)
    {
        spdlog::debug("Use old mask and geometric segment");
        p_tracker_payload = std::make_unique<TrackerPayload>(m_curr_timestamp, *p_input_frame, *mp_prev_masked_image);
    }
    else
    {
        MaskedImage::UniquePtr p_masked_image;
        if(!syncQueue<MaskedImage::UniquePtr>(m_curr_timestamp, mp_masked_image_queue, &p_masked_image))
        {
            spdlog::error("Missing masked image with requested timestamp: {}", m_curr_timestamp);
            return nullptr;
        }
        if(!p_masked_image)
        {
            spdlog::error("Module: {} {} returned null", name_id_, mp_masked_image_queue->queue_id_);
            return nullptr;
        }
        p_tracker_payload = std::make_unique<TrackerPayload>(m_curr_timestamp, *p_input_frame, *p_masked_image);
        mp_prev_masked_image = std::move(p_masked_image);
    }
    if(!p_tracker_payload)
    {
        spdlog::error("Unable to create TrackerPayload");
        return nullptr;
    }
    auto duration = Timer::toc(start_time).count();
    spdlog::info("Processed tracker payload: {}, took {} ms", m_curr_timestamp, duration);

    return p_tracker_payload;
}

Tracker::OutputUniquePtr Tracker::run_once(Tracker::InputUniquePtr p_input)
{
    if(m_first_run)
    {
        CheckCuda(cudaSetDevice(0));
        m_first_run = false;
        open3d::camera::PinholeCameraIntrinsic intrinsic(p_input->get_frame().get_intrinsics());
        mc_intrinsic.SetIntrinsics(intrinsic);
    }

    using namespace open3d;

    auto start_time = Timer::tic();

    auto curr_depth_image = p_input->get_frame().get_depth();
    auto curr_color_image = p_input->get_frame().get_color();

    mc_curr_depth_raw.Upload(curr_depth_image);
    mc_curr_color.Upload(curr_color_image);
    mc_curr_depth_raw.Bilateral(mc_curr_depth_filt);

    mc_curr_vertex_map = mc_curr_depth_filt.GetVertexMap(mc_intrinsic);
    mc_curr_normal_map = mc_curr_vertex_map.GetNormalMap();

    std::shared_ptr<geometry::Image> p_depth = mc_curr_depth_filt.DownloadImage();
    std::shared_ptr<geometry::Image> p_vertex_map = mc_curr_vertex_map.DownloadImage();
    std::shared_ptr<geometry::Image> p_normal_map = mc_curr_normal_map.DownloadImage();

    spdlog::info("Bilateral filter took: {} ms", Timer::toc(start_time).count());
    cv::Mat download_image(p_depth->height_, p_depth->width_, CV_32FC1, p_depth->data_.data());
    cv::Mat vertex_map(p_vertex_map->height_, p_vertex_map->width_, CV_32FC3, p_vertex_map->data_.data());
    cv::Mat normal_map(p_normal_map->height_, p_normal_map->width_, CV_32FC3, p_normal_map->data_.data());

    cv::imshow("Bilateral", download_image);
    cv::imshow("Vertex map", vertex_map);
    cv::imshow("Normal map", normal_map);
    cv::waitKey(2);

    return nullptr;
}


/* Frame::UniquePtr Tracker::getInput() */
/* { */
/*     Frame::UniquePtr input_frame = nullptr; */
/*     bool queue_state = false; */
/*     queue_state = m_input_frame_queue.popBlocking(input_frame); */
/*     if (queue_state) */
/*         return input_frame; */
/*     else */
/*         spdlog::error("Input Frame Queue is down"); */

/*     return nullptr; */
/* } */

/* bool Tracker::process(void)*/
/* {*/
/*     mp_current_frame = getInput();*/

/*     // TODO: Wait for mapping thread to provide map vertices and normals*/
/*     // TODO: Bilateral filter the depth map*/
/*     // TODO: Compute frame vertices and normals*/
/*     // TODO: ICP odometry*/
/*     if (mp_current_frame) {*/
/*         // If frame does not have segmentation push data to transporter*/


/*         bool is_keyframe = mp_current_frame->is_keyframe();*/
/*         if (!mp_current_frame->has_segmentation()) {*/
/*             // Transfer ownership entirely to image transporter?*/
/*             m_transport_callback(std::move(mp_current_frame));*/
/*             spdlog::debug("Called m_transport_callback");*/
/*         }*/

/*         //TEST*/
/*         if(is_keyframe)*/
/*         {*/
/*             MaskedImage::UniquePtr p_masked_image;*/
/*             bool q_state = m_input_mask_queue.popBlocking(p_masked_image);*/

/*             if (q_state) {*/
/*                 for (const auto &label : p_masked_image->labels) spdlog::info("Label {}", label);*/
/*             }*/
/*         }*/


        /* //TODO: Something wrong with obtained transform owing to memory */
        /* std::shared_ptr<oslam::Odometry> p_odometry = mp_current_frame->odometry(mp_prev_frame);
         */

        /* if (mp_prev_frame) */
        /*     mp_current_frame->set_pose(mp_prev_frame->get_pose() * p_odometry->transform); */
        /* spdlog::debug("Frame current pose: \n{}\n", mp_current_frame->get_pose()); */

        /* /1* spdlog::debug("Frame object list size: {}", mp_current_frame->mv_object_rgbd.size());
         * *1/ */

        /* /1* if (mp_current_frame->mv_object_rgbd.size()) { *1/ */
        /* /1*     unsigned int k = 0; *1/ */
        /* /1*     for (const auto &label : mp_current_frame->m_labels) { *1/ */
        /* /1*         auto object_rgbd = mp_current_frame->mv_object_rgbd.at(k); *1/ */
        /* /1*         auto score = mp_current_frame->m_scores.at(k); *1/ */
        /* /1*         auto p_object = Object(object_rgbd, label, score,
         * mp_rgbd_dataset->intrinsic); *1/ */
        /* /1*         k++; *1/ */
        /* /1*     } *1/ */
        /* /1* } *1/ */

/*         mp_prev_frame = std::move(mp_current_frame);*/
/*         m_curr_frame_id++;*/
/*     }*/
/*     return true;*/
/* }*/

}// namespace oslam
