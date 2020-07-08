/******************************************************************************
 * File:             tracker_payload.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Tracker Payload
 *****************************************************************************/
#ifndef OSLAM_TRACKER_PAYLOAD_H
#define OSLAM_TRACKER_PAYLOAD_H

/* #include <gtsam/geometry/Pose3.h> */

#include "frame.h"
#include "instance_image.h"
#include "utils/pipeline_payload.h"

namespace oslam
{
    /*! \class TrackerInput
     *  \brief Brief class description
     *
     *  Detailed description
     */
    struct TrackerInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(TrackerInput);
        TrackerInput(
            const Timestamp timestamp,
            const Frame& r_frame,
            const cv::Mat& r_global_color_map,
            const cv::Mat& r_global_vertices,
            const cv::Mat& r_global_normals)
            : PipelinePayload(timestamp),
              m_frame(r_frame),
              m_global_color_map(r_global_color_map),
              m_global_vertices(r_global_vertices),
              m_global_normals(r_global_normals)
        {
        }
        ~TrackerInput() = default;

        const Frame m_frame;
        const cv::Mat m_global_color_map;
        const cv::Mat m_global_vertices;
        const cv::Mat m_global_normals;
    };

    enum class TrackerStatus
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
        TrackerOutput(
            const Timestamp timestamp,
            const TrackerStatus& r_tracker_status,
            const Frame& r_frame,
            const Eigen::Matrix4d& r_relative_camera_pose,
            const Eigen::Matrix6d& r_information_matrix)
            : PipelinePayload(timestamp),
            m_tracker_status(r_tracker_status),
            m_frame(r_frame),
            m_relative_camera_pose(r_relative_camera_pose),
            m_information_matrix(r_information_matrix)
        {
        }
        ~TrackerOutput() = default;
       public:
        const TrackerStatus m_tracker_status;
        const Frame m_frame;
        const Eigen::Matrix4d m_relative_camera_pose;
        const Eigen::Matrix6d m_information_matrix;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_TRACKER_PAYLOAD_H */
