/******************************************************************************
* File:             segmenter-payload.h
*
* Author:           Akash Sharma
* Created:          06/19/20
* Description:      Segmenter payload definitions
*****************************************************************************/
#ifndef OSLAM_SEGMENTER_PAYLOAD_H
#define OSLAM_SEGMENTER_PAYLOAD_H

#include "utils/macros.h"
#include "utils/pipeline_payload.h"
#include "utils/macros.h"

#include "frame.h"

namespace oslam {

    struct SegmenterInput : public PipelinePayload
    {
        OSLAM_POINTER_TYPEDEFS(SegmenterInput);
        OSLAM_DELETE_COPY_CONSTRUCTORS(SegmenterInput);

        SegmenterInput(Timestamp timestamp, const Frame& r_frame, const MaskedImage& r_masked_image, Timestamp prev_maskframe_timestamp)
            : PipelinePayload(timestamp), m_frame(r_frame), m_masked_image(r_masked_image), m_prev_maskframe_timestamp(prev_maskframe_timestamp)
        {}

        public:
        const Frame m_frame;
        const MaskedImage m_masked_image;
        const Timestamp m_prev_maskframe_timestamp;

    };
}
#endif /* ifndef OSLAM_SEGMENTER_PAYLOAD_H */
