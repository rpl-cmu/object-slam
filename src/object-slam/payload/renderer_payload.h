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

#include "object-slam/utils/pipeline_payload.h"
#include "object-slam/utils/types.h"

#include "object-slam/payload/display_payload.h"
#include "object-slam/struct/frame.h"

namespace oslam
{
    using namespace open3d;

    enum class MapperStatus
    {
        VALID   = 0,
        INVALID = 1
    };

    struct ObjectRender
    {
       public:
        OSLAM_POINTER_TYPEDEFS(ObjectRender);
        ObjectRender(cv::Mat color_map, cv::Mat vertex_map, cv::Mat normal_map)
            : color_map_(std::move(color_map)), vertex_map_(std::move(vertex_map)), normal_map_(std::move(normal_map))
        {
        }

        ~ObjectRender() = default;
        cv::Mat color_map_;
        cv::Mat vertex_map_;
        cv::Mat normal_map_;
    };

    using ObjectRenders          = std::unordered_map<ObjectId, ObjectRender>;
    using ObjectRendersUniquePtr = std::unique_ptr<ObjectRenders>;

    struct RendererInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererInput);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RendererInput(Timestamp timestamp,
                      const MapperStatus& mapper_status,
                      ObjectRendersUniquePtr object_renders,
                      const Frame& frame,
                      const Eigen::Matrix4d& camera_pose)
            : PipelinePayload(timestamp),
              mapper_status_(mapper_status),
              object_renders_(std::move(object_renders)),
              frame_(frame),
              camera_pose_(camera_pose)
        {
        }

        ~RendererInput() = default;

        const MapperStatus mapper_status_;
        const ObjectRendersUniquePtr object_renders_;
        const Frame frame_;
        const Eigen::Matrix4d camera_pose_;
    };

    struct RendererOutput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererOutput);

        RendererOutput(Timestamp timestamp, ObjectRender::UniquePtr background_render)
            : PipelinePayload(timestamp),
              background_render_(std::move(background_render))
        {
        }

        ~RendererOutput() = default;

        ObjectRender::UniquePtr background_render_;
        std::map<std::string, WidgetPtr> widgets_map_{};
    };

}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_PAYLOAD_H */
