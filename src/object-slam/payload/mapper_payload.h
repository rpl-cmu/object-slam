/******************************************************************************
 * File:             mapper_payload.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Mapper payload data
 *****************************************************************************/
#ifndef OSLAM_MAPPER_PAYLOAD_H
#define OSLAM_MAPPER_PAYLOAD_H

#include "object-slam/utils/pipeline_payload.h"

#include "object-slam/struct/frame.h"
#include "object-slam/struct/instance_image.h"
#include "object-slam/payload/tracker_payload.h"

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
            const TrackerStatus& tracker_status,
            const Frame& frame,
            const InstanceImages& instance_images,
            const Eigen::Matrix4d& relative_camera_pose,
            const Eigen::Matrix6d& information_matrix)
            : PipelinePayload(timestamp),
              prev_maskframe_timestamp_(prev_maskframe_timestamp),
              tracker_status_(tracker_status),
              frame_(frame),
              instance_images_(instance_images),
              relative_camera_pose_(relative_camera_pose),
              information_matrix_(information_matrix)
        {
        }

       public:
        const Timestamp prev_maskframe_timestamp_;
        const TrackerStatus tracker_status_;
        const Frame frame_;
        const InstanceImages instance_images_;
        const Eigen::Matrix4d relative_camera_pose_;
        const Eigen::Matrix6d information_matrix_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_PAYLOAD_H */
