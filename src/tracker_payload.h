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
       public:
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

    struct TrackerOutput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(TrackerOutput);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        TrackerOutput(const Timestamp timestamp, const Timestamp prev_maskframe_timestamp,
                      const TrackerStatus& r_tracker_status, const Frame& r_frame, const InstanceImages& r_instance_images,
                      const Eigen::Matrix4d& r_relative_camera_pose, const Eigen::Matrix6d& r_information_matrix)
            : PipelinePayload(timestamp),
              m_prev_maskframe_timestamp(prev_maskframe_timestamp),
              m_tracker_status(r_tracker_status),
              m_frame(r_frame),
              m_instance_images(r_instance_images),
              m_relative_camera_pose(r_relative_camera_pose),
              m_information_matrix(r_information_matrix)
        {
        }

       public:
        const Timestamp m_prev_maskframe_timestamp;
        const TrackerStatus m_tracker_status;
        const Frame& m_frame;
        const InstanceImages& m_instance_images;
        const Eigen::Matrix4d m_relative_camera_pose;
        const Eigen::Matrix6d m_information_matrix;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_TRACKER_PAYLOAD_H */
