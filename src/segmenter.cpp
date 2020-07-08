/******************************************************************************
* File:             segmenter.cpp
*
* Author:           Akash Sharma
* Created:          06/19/20
* Description:      Segmentation thread
*****************************************************************************/
#include "segmenter.h"
#include <spdlog/spdlog.h>

namespace oslam {

    Segmenter::Segmenter(FrameQueue* p_frame_queue, MaskedImageQueue* p_masked_image_queue)
        : MIMO("Segmenter"), mp_frame_queue(p_frame_queue), mp_masked_image_queue(p_masked_image_queue)
    {}

    Segmenter::InputUniquePtr Segmenter::get_input_packet()
    {
        auto start_time = Timer::tic();

        Frame::UniquePtr p_frame;
        bool frame_queue_state = mp_frame_queue->popBlocking(p_frame);
        if (!frame_queue_state)
        {
            spdlog::error("Module: {} {} is down", name_id_, mp_frame_queue->queue_id_);
            return nullptr;
        }
        if (!p_frame)
        {
            spdlog::error("Module: {} {} returned null", name_id_, mp_frame_queue->queue_id_);
            return nullptr;
        }

        m_curr_timestamp = p_frame->m_timestamp;
        const bool is_maskframe = p_frame->is_maskframe();

        Segmenter::InputUniquePtr p_segmenter_payload;
        if (!is_maskframe)
        {
            spdlog::debug("SegmenterInputPayload: frame + old mask");
            //! Simply use the previous masked image to create tracker payload
            //! TODO(Akash): This creates a copy of the data. Should i use a pointer here?
            p_segmenter_payload = std::make_unique<SegmenterInput>(m_curr_timestamp, *p_frame, *mp_prev_masked_image, m_prev_maskframe_timestamp);
        }
        else
        {
            MaskedImage::UniquePtr p_masked_image;
            //! Try to synchronize the Masked Image queue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<MaskedImage::UniquePtr>(m_curr_timestamp, mp_masked_image_queue, &p_masked_image))
            {
                spdlog::error("Missing masked image with requested timestamp: {}", m_curr_timestamp);
                return nullptr;
            }
            if (!p_masked_image)
            {
                spdlog::error("Module: {} {} returned null", name_id_, mp_masked_image_queue->queue_id_);
                return nullptr;
            }
            p_segmenter_payload =
                std::make_unique<SegmenterInput>(m_curr_timestamp, *p_frame, *p_masked_image, m_curr_timestamp);
            mp_prev_masked_image       = std::move(p_masked_image);
            m_prev_maskframe_timestamp = m_curr_timestamp;
        }
        if(!p_segmenter_payload)
        {
            spdlog::error("Unable to create SegmenterInput: {}", name_id_);
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::info("Processed tracker payload: {}, took {} ms", m_curr_timestamp, duration);
        return p_segmenter_payload;
    }

    Segmenter::OutputUniquePtr Segmenter::run_once(Segmenter::InputUniquePtr p_input)
    {
        const Frame curr_frame = p_input->m_frame;
        const MaskedImage curr_masked_image = p_input->m_masked_image;

        //!TODO(Akash): Segmenter can run only after initial transformation is obtained, to transform and project semantic segmentation


    }

} // namespace oslam
