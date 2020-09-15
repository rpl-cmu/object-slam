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

    struct RendererInput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererInput);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RendererInput(Timestamp timestamp,
                      const MapperStatus& mapper_status,
                      const Renders& object_renders,
                      const Frame& frame)
            : PipelinePayload(timestamp), mapper_status_(mapper_status), object_renders_(object_renders), frame_(frame)
        {
        }

        ~RendererInput() = default;

        const MapperStatus mapper_status_;
        const Renders object_renders_;
        const Frame frame_;
    };

    struct RendererOutput : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(RendererOutput);

        RendererOutput(Timestamp timestamp, Render::UniquePtr background_render)
            : PipelinePayload(timestamp), background_render_(std::move(background_render))
        {
        }

        ~RendererOutput() = default;

        Render::UniquePtr background_render_;
        std::map<std::string, WidgetPtr> widgets_map_{};
    };

}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_PAYLOAD_H */
