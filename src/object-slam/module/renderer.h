/******************************************************************************
 * File:             renderer.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Renderer which raycasts vertices and normals / generates visualization
 *****************************************************************************/
#ifndef OSLAM_RENDERER_H
#define OSLAM_RENDERER_H

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"

#include "object-slam/payload/renderer_payload.h"
#include "object-slam/struct/map.h"

namespace oslam
{
    /*! \class Renderer
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Renderer : public SISOPipelineModule<RendererInput, RendererOutput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Renderer);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Renderer);

        using SISO = SISOPipelineModule<RendererInput, RendererOutput>;

        Renderer(Map::Ptr map, InputQueue* input_queue, OutputQueue* output_queue);
        virtual ~Renderer() = default;

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

       protected:
        Timestamp curr_timestamp_ = 0;

        Map::Ptr map_;
        cuda::ImageCuda<float, 3> model_vertices_cuda_;
        cuda::ImageCuda<float, 3> model_normals_cuda_;
        cuda::ImageCuda<uchar, 3> model_colors_cuda_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_H */
