/******************************************************************************
* File:             tracker_payload.h
*
* Author:           Akash Sharma
* Created:          05/16/20
* Description:      Tracker Payload
*****************************************************************************/
#ifndef OSLAM_TRACKER_PAYLOAD_H
#define OSLAM_TRACKER_PAYLOAD_H

#include "frame.h"
#include "masked_image.h"
#include "utils/pipeline_payload.h"

namespace oslam {
/*! \class TrackerPayload
 *  \brief Brief class description
 *
 *  Detailed description
 */
class TrackerPayload : public PipelinePayload
{
public:
    TrackerPayload(const Timestamp timestamp, const Frame& r_frame, const MaskedImage& r_masked_image)
        : PipelinePayload(timestamp), m_frame(r_frame), m_masked_image(r_masked_image)
    {}
    ~TrackerPayload() = default;

    inline Frame get_frame(void) const { return m_frame; }
    inline MaskedImage get_masked_image(void) const { return m_masked_image; }

private:
    const Frame m_frame;
    const MaskedImage m_masked_image;
    //!TODO(Akash): Potentially this will contain map vertices / normals
};
}
#endif /* ifndef OSLAM_TRACKER_PAYLOA */
