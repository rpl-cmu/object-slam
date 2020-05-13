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
    m_input_frame_queue("InputFrameQueue"), m_input_mask_queue("InputMaskQueue"),
    mp_current_frame(nullptr), mp_prev_frame(nullptr)
{}

Tracker::~Tracker() {}

Frame::UniquePtr Tracker::getInput()
{
    Frame::UniquePtr input_frame = nullptr;
    bool queue_state = false;
    queue_state = m_input_frame_queue.popBlocking(input_frame);
    if (queue_state)
        return input_frame;
    else
        spdlog::error("Input Frame Queue is down");

    return nullptr;
}

bool Tracker::process(void)
{
    mp_current_frame = getInput();

    // TODO: Wait for mapping thread to provide map vertices and normals
    // TODO: Bilateral filter the depth map
    // TODO: Compute frame vertices and normals
    // TODO: ICP odometry
    if (mp_current_frame) {
        // If frame does not have segmentation push data to transporter


        bool is_keyframe = mp_current_frame->is_keyframe();
        if (!mp_current_frame->has_segmentation()) {
            // Transfer ownership entirely to image transporter?
            m_transport_callback(std::move(mp_current_frame));
            spdlog::debug("Called m_transport_callback");
        }

        //TEST
        if(is_keyframe)
        {
            MaskedImage::UniquePtr p_masked_image;
            bool q_state = m_input_mask_queue.popBlocking(p_masked_image);

            if (q_state) {
                for (const auto &label : p_masked_image->labels) spdlog::info("Label {}", label);
            }
        }


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

        mp_prev_frame = std::move(mp_current_frame);
        m_curr_frame_id++;
    }
    return true;
}

}// namespace oslam
