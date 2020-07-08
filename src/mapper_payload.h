/******************************************************************************
 * File:             mapper_payload.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Mapper payload data
 *****************************************************************************/
#ifndef OSLAM_MAPPER_PAYLOAD_H
#define OSLAM_MAPPER_PAYLOAD_H

#include "frame.h"
#include "instance_image.h"
#include "tracker_payload.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
    struct MapperInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(MapperInput);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MapperInput(
            const Timestamp timestamp,
            const Timestamp prev_maskframe_timestamp,
            const TrackerStatus& r_tracker_status,
            const Frame& r_frame,
            const InstanceImages& r_instance_images,
            const Eigen::Matrix4d& r_relative_camera_pose,
            const Eigen::Matrix6d& r_information_matrix)
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
        const Frame m_frame;
        const InstanceImages m_instance_images;
        const Eigen::Matrix4d m_relative_camera_pose;
        const Eigen::Matrix6d m_information_matrix;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_PAYLOAD_H */
