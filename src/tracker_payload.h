/******************************************************************************
 * File:             tracker_payload.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Tracker Payload
 *****************************************************************************/
#ifndef OSLAM_TRACKER_PAYLOAD_H
#define OSLAM_TRACKER_PAYLOAD_H

#include <gtsam/geometry/Pose3.h>

#include "frame.h"
#include "instance_image.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
    /*! \class TrackerInputPayload
     *  \brief Brief class description
     *
     *  Detailed description
     */

    struct TrackerInput : public PipelinePayload
    {
        using InstanceImages = std::vector<InstanceImage>;
        OSLAM_POINTER_TYPEDEFS(TrackerInput);
        TrackerInput(const Timestamp timestamp, const Timestamp prev_maskframe_timestamp, const Frame& r_frame,
                     const InstanceImages& r_instance_images)
            : PipelinePayload(timestamp),
              m_prev_maskframe_timestamp(prev_maskframe_timestamp),
              m_frame(r_frame),
              m_instance_images(r_instance_images)
        {
        }
        ~TrackerInput() = default;

        const Timestamp m_prev_maskframe_timestamp;
        const Frame m_frame;
        const InstanceImages m_instance_images;
    };
    //! TODO:(Akash) Figure out Eigen issue with using alignment in GTSAM vs no-alignment in Open3D!!!

    enum TrackerStatus
    {
        VALID,
        INVALID,
        DISABLED
    };

    struct TrackerOutputPayload : public PipelinePayload
    {
       public:
        TrackerOutputPayload(const Timestamp timestamp, const TrackerStatus& r_tracker_status,
                             const gtsam::Pose3& r_T_camera_2_world)
            : PipelinePayload(timestamp), m_tracker_status(r_tracker_status), m_T_camera_2_world(r_T_camera_2_world)
        {
        }

       public:
        const TrackerStatus m_tracker_status;
        const gtsam::Pose3 m_T_camera_2_world;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_TRACKER_PAYLOAD_H */
