/******************************************************************************
 * File:             renderer.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Renderer which raycasts vertices and normals / generates visualization
 *****************************************************************************/
#ifndef OSLAM_RENDERER_H
#define OSLAM_RENDERER_H

#include "utils/macros.h"
#include "utils/pipeline_module.h"

#include "renderer_payload.h"
#include "map.h"

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

        Renderer(InputQueue* p_input_queue, OutputQueue* p_output_queue);
        virtual ~Renderer() = default;

        virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

       protected:

        Timestamp m_curr_timestamp = 0;

        GlobalMap& mr_global_map;
        cuda::ImageCuda<float, 3> mc_current_global_vertices;
        cuda::ImageCuda<float, 3> mc_current_global_normals;
        cuda::ImageCuda<uchar, 3> mc_current_global_colors;

    };
}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_H */
