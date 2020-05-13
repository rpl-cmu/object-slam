/******************************************************************************
 * File:             tracker.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker thread
 *****************************************************************************/
#include "tracker.h"

#include <memory>

namespace oslam {

Tracker::Tracker(std::shared_ptr<oslam::ImageTransporter> p_image_transport)
  : Thread("TrackerThread"), mp_image_transport(p_image_transport), m_curr_frame_id(0),
    m_rgbd_data_queue("RGBDdataQueue"), mp_current_frame(nullptr), mp_prev_frame(nullptr)
{}

Tracker::~Tracker() {}

bool Tracker::process(void)
{
    std::unique_ptr<RGBDdata> p_data = nullptr;
    bool queue_state = false;

    queue_state = m_rgbd_data_queue.popBlocking(p_data);

    if (!queue_state) spdlog::error("RGBD data queue is down");

    // TODO: Why not just create frame object directly from the datareader?
    mp_current_frame = std::make_shared<oslam::Frame>(m_curr_frame_id, *p_data);

    // TODO: Wait for mapping thread to provide map vertices and normals
    // TODO: Bilateral filter the depth map
    // TODO: Compute frame vertices and normals
    // TODO: ICP odometry
    std::shared_ptr<oslam::Odometry> p_odometry = mp_current_frame->odometry(mp_prev_frame);

    if (mp_prev_frame)
        mp_current_frame->set_pose(mp_prev_frame->get_pose() * p_odometry->transform);

    spdlog::debug("Frame current pose: \n{}\n", mp_current_frame->get_pose());

    /* spdlog::debug("Frame object list size: {}", mp_current_frame->mv_object_rgbd.size()); */

    /* if (mp_current_frame->mv_object_rgbd.size()) { */
    /*     unsigned int k = 0; */
    /*     for (const auto &label : mp_current_frame->m_labels) { */
    /*         auto object_rgbd = mp_current_frame->mv_object_rgbd.at(k); */
    /*         auto score = mp_current_frame->m_scores.at(k); */
    /*         auto p_object = Object(object_rgbd, label, score, mp_rgbd_dataset->intrinsic); */
    /*         k++; */
    /*     } */
    /* } */

    mp_prev_frame = mp_current_frame;
    m_curr_frame_id++;

    return true;
}

}// namespace oslam
