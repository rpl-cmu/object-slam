/******************************************************************************
 * File:             renderer.h
 *
 * Author:           Akash Sharma
 * Created:          07/07/20
 * Description:      Renderer which raycasts vertices and normals / generates visualization
 *****************************************************************************/
#ifndef OSLAM_RENDERER_H
#define OSLAM_RENDERER_H

#include <deque>
#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"

#include "object-slam/payload/display_payload.h"
#include "object-slam/payload/renderer_payload.h"
#include "object-slam/struct/map.h"

namespace oslam
{
    /*! \class Renderer
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Renderer : public SIMOPipelineModule<RendererInput, RendererOutput>
    {
       public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        OSLAM_POINTER_TYPEDEFS(Renderer);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Renderer);
        OSLAM_DELETE_MOVE_CONSTRUCTORS(Renderer);

        using SIMO = SIMOPipelineModule<RendererInput, RendererOutput>;

        Renderer(const Map::Ptr& map, InputQueue* input_queue);
        virtual ~Renderer() = default;

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

       private:
        static std::vector<cv::Affine3d> fillCameraTrajectory(const PoseTrajectory& camera_trajectory);
        WidgetPtr render3dTrajectory(const std::vector<cv::Affine3d>& camera_trajectory_3d);
        WidgetPtr render3dFrustumTraj(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                      const Eigen::Matrix3d& intrinsic_matrix,
                                      const size_t& num_prev_frustums);
        WidgetPtr render3dFrustumWithColorMap(const std::vector<cv::Affine3d>& camera_trajectory_3d,
                                              const Eigen::Matrix3d& intrinsic_matrix,
                                              const cv::Mat& color_map);
        void renderFrustumPlanes(const PointPlanes& point_planes, std::map<std::string, WidgetPtr>& widget_map);

        void renderObjectCubes(const ObjectBoundingBoxes& object_bboxes, std::map<std::string, WidgetPtr>& widget_map);
        void renderObjectMeshes(const IdToObjectMesh& object_meshes, std::map<std::string, WidgetPtr>& widget_map);
        Timestamp curr_timestamp_ = 0;

        Map::Ptr map_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_RENDERER_H */
