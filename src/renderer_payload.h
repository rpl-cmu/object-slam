/******************************************************************************
 * File:             renderer_payload.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Payload definitions for Renderer
 *****************************************************************************/
#ifndef OSLAM_RENDERER_PAYLOAD_H
#define OSLAM_RENDERER_PAYLOAD_H

#include <Cuda/Geometry/ImageCuda.h>

#include <opencv2/core/types.hpp>

#include "frame.h"
#include "utils/pipeline_payload.h"
#include "utils/types.h"

namespace oslam
{
    using namespace open3d;

    enum class MapperStatus
    {
        VALID   = 0,
        INVALID = 1
    };

    struct RendererInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererInput);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RendererInput(
            Timestamp timestamp,
            const MapperStatus& mapper_status,
            const Frame& frame,
            const Eigen::Matrix4d& camera_pose)
            : PipelinePayload(timestamp), mapper_status_(mapper_status), frame_(frame), camera_pose_(camera_pose)
        {
        }

        ~RendererInput() = default;

        const MapperStatus mapper_status_;
        const Frame frame_;
        const Eigen::Matrix4d camera_pose_;
    };

    struct RendererOutput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererOutput);

        RendererOutput(Timestamp timestamp, const cv::Mat& colors, const cv::Mat& vertices, const cv::Mat& normals)
            : PipelinePayload(timestamp), colors_(colors), vertices_(vertices), normals_(normals)
        {
        }

        ~RendererOutput() = default;

        const cv::Mat colors_;
        const cv::Mat vertices_;
        const cv::Mat normals_;
    };

}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_PAYLOAD_H */
