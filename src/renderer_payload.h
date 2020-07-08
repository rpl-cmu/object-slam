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
            const MapperStatus& r_mapper_status,
            const Frame& r_frame,
            const Eigen::Matrix4d& r_camera_pose)
            : PipelinePayload(timestamp), m_mapper_status(r_mapper_status), m_frame(r_frame), m_camera_pose(r_camera_pose)
        {
        }

        ~RendererInput() = default;

        const MapperStatus m_mapper_status;
        const Frame m_frame;
        const Eigen::Matrix4d m_camera_pose;
    };

    struct RendererOutput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererOutput);

        RendererOutput(Timestamp timestamp, const cv::Mat& r_colors, const cv::Mat& r_vertices, const cv::Mat& r_normals)
            : PipelinePayload(timestamp), m_colors(r_colors), m_vertices(r_vertices), m_normals(r_normals)
        {
        }

        ~RendererOutput() = default;

        const cv::Mat m_colors;
        const cv::Mat m_vertices;
        const cv::Mat m_normals;
    };

}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_PAYLOAD_H */
