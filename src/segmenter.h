/******************************************************************************
 * File:             segmenter.h
 *
 * Author:           Akash Sharma
 * Created:          06/19/20
 * Description:      Segmentation thread which prepares TrackerInputPayload
 *****************************************************************************/
#ifndef OSLAM_SEGMENTER_H
#define OSLAM_SEGMENTER_H

#include "segmenter_payload.h"
#include "utils/macros.h"
#include "utils/pipeline_module.h"
#include "utils/thread_safe_queue.h"

#include "frame.h"
#include "masked_image.h"

namespace oslam
{
    /*! \class Segmenter
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Segmenter : public MIMOPipelineModule<SegmenterInput, SegmenterInput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Segmenter);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Segmenter);

        using MIMO             = MIMOPipelineModule<SegmenterInput, SegmenterInput>;
        using MaskedImageQueue = ThreadSafeQueue<MaskedImage::UniquePtr>;
        using FrameQueue       = ThreadSafeQueue<Frame::UniquePtr>;

        explicit Segmenter(MaskedImageQueue* p_masked_image_queue, FrameQueue* p_frame_queue);
        virtual ~Segmenter() = default;

        //! \brief: Reads in the SegmenterInput payload, adds depth segmentation
        //! and generates the SegmenterOutput payload
        //! being a MIMO package, the SegmenterOutput payload is transferred by callbacks to requisite modules
        virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

       private:
        //! \brief: override the get_input_packet interface
        //! Segmenter reads from the FrameQueue (prepared by DataReader) and MaskedImageQueue (prepared by ImageTransporter)
        //! Synchronizes the packets and generates a SegmenterInput payload
        virtual InputUniquePtr get_input_packet() override;

        FrameQueue* mp_frame_queue;
        MaskedImageQueue* mp_masked_image_queue;

        MaskedImage::UniquePtr mp_prev_masked_image;

        Timestamp m_curr_timestamp;
        Timestamp m_prev_maskframe_timestamp;

    };
}  // namespace oslam

#endif /* ifndef OSLAM_SEGMENTER_H */
