/******************************************************************************
 * File:             object.hpp
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      TSDFObject implementation
 *****************************************************************************/
#include "tsdf_object.h"

#include <Cuda/Common/TransformCuda.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Cuda/Geometry/PointCloudCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <tuple>
#include <vector>

namespace oslam
{
    TSDFObject::TSDFObject(const ObjectId &id,
                           const Frame &frame,
                           const InstanceImage &instance_image,
                           const Eigen::Matrix4d &camera_pose,
                           int resolution)
        : id_(id),
          resolution_(resolution),
          instance_image_(instance_image),
          pose_(camera_pose),
          intrinsic_(frame.intrinsic_),
          intrinsic_cuda_(intrinsic_)
    {
        auto color        = frame.color_;
        auto object_depth = frame.depth_.clone();
        object_depth.setTo(0, ~instance_image_.bbox_mask_);

        // Construct Object RGBD to obtain center and estimate object size
        open3d::cuda::RGBDImageCuda object_rgbd;
        object_rgbd.Upload(object_depth, color);
        open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexRaw, frame.width_ * frame.height_);
        object_point_cloud.Build(object_rgbd, intrinsic_cuda_);

        // Translate the object pose to point cloud center
        // T_wo = T_wc * T_co
        pose_.block<3, 1>(0, 3) = object_point_cloud.GetCenter();
        pose_                   = camera_pose * pose_;

        // Appropriately size the voxel_length of volume for appropriate resolution of the object
        object_max_pt_     = Eigen::Affine3d(pose_) * object_point_cloud.GetMaxBound();
        object_min_pt_     = Eigen::Affine3d(pose_) * object_point_cloud.GetMinBound();
        float voxel_length = VOLUME_SIZE_SCALE * float((object_max_pt_ - object_min_pt_).maxCoeff() / resolution);

        spdlog::debug("Volume pose\n {}", pose_);
        spdlog::debug("Voxel length: {}", voxel_length);

        open3d::cuda::TransformCuda object_pose_cuda;
        object_pose_cuda.FromEigen(camera_pose);

        // truncation distance = 4 * voxel length
        // Allocate lower memory since we will create multiple TSDF objects
        int BUCKET_COUNT   = 2000;
        int VALUE_CAPACITY = 4500;
        if (instance_image.label_ == 0)
        {
            BUCKET_COUNT   = 2000;
            VALUE_CAPACITY = 10000;
            spdlog::debug("Created new background instance");
        }
        else
        {
            spdlog::debug("Created new object instance");
        }
        volume_ = cuda::ScalableTSDFVolumeCuda(SUBVOLUME_RES,
                                               voxel_length,
                                               TSDF_TRUNCATION_SCALE * voxel_length,
                                               object_pose_cuda,
                                               BUCKET_COUNT,
                                               VALUE_CAPACITY);
    }

    void TSDFObject::integrate(const Frame &frame, const InstanceImage &instance_image, const Eigen::Matrix4d &camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integration(mutex_);

        auto color        = frame.color_;
        auto object_depth = frame.depth_.clone();
        object_depth.setTo(0, ~instance_image.bbox_mask_);

        open3d::cuda::RGBDImageCuda object_rgbd_cuda;
        object_rgbd_cuda.Upload(object_depth, color);

        open3d::cuda::TransformCuda camera_to_world_cuda;
        camera_to_world_cuda.FromEigen(camera_pose);

        open3d::cuda::ImageCuda<uchar, 1> mask_cuda;
        mask_cuda.Upload(instance_image.maskb_);

        volume_.Integrate(
            object_rgbd_cuda, intrinsic_cuda_, camera_to_world_cuda, static_cast<int>(frame.timestamp_), mask_cuda);

#ifdef OSLAM_DEBUG_VIS
        if (frame.timestamp_ % 500 == 0)
        {
            volume_.GetAllSubvolumes();
            open3d::cuda::ScalableMeshVolumeCuda mesher(
                open3d::cuda::VertexWithColor, 16, volume_.active_subvolume_entry_array_.size(), 2000000, 4000000);
            mesher.MarchingCubes(volume_);
            auto mesh = mesher.mesh().Download();
            open3d::visualization::DrawGeometries({ mesh }, "Mesh after integration");
            mesher.Release();
        }
#endif
    }

    void TSDFObject::raycast(open3d::cuda::ImageCuda<float, 3> &vertex,
                             open3d::cuda::ImageCuda<float, 3> &normal,
                             open3d::cuda::ImageCuda<uchar, 3> &color,
                             const Eigen::Matrix4d &camera_pose)
    {
        std::scoped_lock<std::mutex> lock_raycast(mutex_);
        open3d::cuda::TransformCuda camera_to_object_cuda;
        camera_to_object_cuda.FromEigen(camera_pose);
        volume_.RayCasting(vertex, normal, color, intrinsic_cuda_, camera_to_object_cuda);
    }

    double TSDFObject::getVisibilityRatio(Timestamp timestamp) const
    {
        int visible_blocks = volume_.GetVisibleSubvolumesCount(static_cast<int>(timestamp), RETROSPECT_VISIBILITY_THRESH);
        int total_blocks   = volume_.GetTotalAllocatedSubvolumesCount();
        spdlog::debug("Visible blocks: {}, total_blocks: {}", visible_blocks, total_blocks);
        return double(visible_blocks) / double(total_blocks);
    }

}  // namespace oslam
