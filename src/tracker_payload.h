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

namespace oslam
{
    /*! \class TrackerInputPayload
     *  \brief Brief class description
     *
     *  Detailed description
     */

    class TrackerInputPayload : public PipelinePayload
    {
       public:
        TrackerInputPayload(const Timestamp timestamp, const Timestamp prev_maskframe_timestamp, const Frame& r_frame,
                            const MaskedImage& r_masked_image)
            : PipelinePayload(timestamp),
              m_prev_maskframe_timestamp(prev_maskframe_timestamp),
              m_frame(r_frame),
              m_masked_image(r_masked_image)
        {
        }
        ~TrackerInputPayload() = default;

        inline Frame get_frame(void) const { return m_frame; }
        inline MaskedImage get_masked_image(void) const { return m_masked_image; }
        inline Timestamp get_prev_maskframe_timestamp(void) const { return m_prev_maskframe_timestamp; }

       private:
        const Timestamp m_prev_maskframe_timestamp;
        const Frame m_frame;
        const MaskedImage m_masked_image;
    };
    //!TODO:(Akash) Figure out Eigen issue with using alignment in GTSAM vs no-alignment in Open3D!!!

    /* enum TrackerStatus */
    /* { */
    /*     VALID, */
    /*     INVALID, */
    /*     DISABLED */
    /* }; */

    /* class TrackerOutputPayload : public PipelinePayload */
    /* { */
    /*    public: */
    /*     TrackerOutputPayload(const Timestamp timestamp, const TrackerStatus& r_tracker_status, */
    /*                          const gtsam::Pose3& r_T_camera_2_world) */
    /*         : PipelinePayload(timestamp), m_tracker_status(r_tracker_status), m_T_camera_2_world(r_T_camera_2_world) */
    /*     { */
    /*     } */

    /*    private: */
    /*     TrackerStatus m_tracker_status; */
    /*     gtsam::Pose3  m_T_camera_2_world; */
    /* }; */
}  // namespace oslam
#endif /* ifndef OSLAM_TRACKER_PAYLOAD_H */
