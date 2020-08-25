/******************************************************************************
 * File:             trackepayload.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Tracker Payload
 *****************************************************************************/
#ifndef OSLAM_TRACKER_PAYLOAD_H
#define OSLAM_TRACKER_PAYLOAD_H

#include "object-slam/utils/pipeline_payload.h"

#include "object-slam/struct/frame.h"
#include "object-slam/struct/instance_image.h"

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
        TrackerInput(const Timestamp timestamp,
                     const Frame& frame,
                     const cv::Mat& model_color_map,
                     const cv::Mat& model_vertices,
                     const cv::Mat& model_normals)
            : PipelinePayload(timestamp),
              frame_(frame),
              model_color_map_(model_color_map),
              model_vertices_(model_vertices),
              model_normals_(model_normals)
        {
        }
        ~TrackerInput() = default;

       public:
        const Frame frame_;
        const cv::Mat model_color_map_;
        const cv::Mat model_vertices_;
        const cv::Mat model_normals_;
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
        TrackerOutput(const Timestamp timestamp,
                      const TrackerStatus& tracker_status,
                      const Frame& frame,
                      const Eigen::Matrix4d& relative_camera_pose,
                      const Eigen::Matrix6d& information_matrix)
            : PipelinePayload(timestamp),
              tracker_status_(tracker_status),
              frame_(frame),
              relative_camera_pose_(relative_camera_pose),
              information_matrix_(information_matrix)
        {
        }
        ~TrackerOutput() = default;

       public:
        const TrackerStatus tracker_status_;
        const Frame frame_;
        const Eigen::Matrix4d relative_camera_pose_;
        const Eigen::Matrix6d information_matrix_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_TRACKER_PAYLOAD_H */
