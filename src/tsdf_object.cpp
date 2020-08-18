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
        open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexWithColor, frame.width_ * frame.height_);
        object_point_cloud.Build(object_rgbd, intrinsic_cuda_);

        // Translate the object pose to point cloud center
        // T_wo = T_wc * T_co
        pose_.block<3, 1>(0, 3) = object_point_cloud.GetCenter();
        pose_                   = camera_pose * pose_;

        // Appropriately size the voxel_length of volume for appropriate resolution of the object
        Eigen::Vector3d object_max = object_point_cloud.GetMaxBound();
        Eigen::Vector3d object_min = object_point_cloud.GetMinBound();
        float voxel_length         = VOLUME_SIZE_SCALE * float((object_max - object_min).maxCoeff() / resolution);

        spdlog::debug("Volume pose\n {}", pose_);
        spdlog::debug("Voxel length: {}", voxel_length);

        open3d::cuda::TransformCuda object_pose_cuda;
        object_pose_cuda.FromEigen(camera_pose);

        // truncation distance = 4 * voxel length
        // Allocate lower memory since we will create multiple TSDF objects
        if (instance_image.label_ == 0)
        {
            constexpr int BUCKET_COUNT   = 2000;
            constexpr int VALUE_CAPACITY = 8000;
            volume_                      = cuda::ScalableTSDFVolumeCuda(SUBVOLUME_RES,
                                                   voxel_length,
                                                   TSDF_TRUNCATION_SCALE * voxel_length,
                                                   object_pose_cuda,
                                                   BUCKET_COUNT,
                                                   VALUE_CAPACITY);
            spdlog::debug("Created new background instance");
            return;
        }
        constexpr int BUCKET_COUNT   = 2000;
        constexpr int VALUE_CAPACITY = 4000;
        volume_                      = cuda::ScalableTSDFVolumeCuda(SUBVOLUME_RES,
                                               voxel_length,
                                               TSDF_TRUNCATION_SCALE * voxel_length,
                                               object_pose_cuda,
                                               BUCKET_COUNT,
                                               VALUE_CAPACITY);
        spdlog::debug("Created new object instance");
    }

    void TSDFObject::integrate(const Frame &frame, const InstanceImage &instance_image, const Eigen::Matrix4d &camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integration(mutex_);

        auto color        = frame.color_;
        auto object_depth = frame.depth_.clone();
        object_depth.setTo(0, ~instance_image.bbox_mask_);

        // For visualization/debug only
#ifdef OSLAM_DEBUG_VIS
        cv::Mat cvt8;
        cv::convertScaleAbs(object_depth, cvt8, 0.25F);
        /* cv::imshow("Integrating Depth", cvt8); */
        /* cv::waitKey(1); */
#endif

        open3d::cuda::RGBDImageCuda object_rgbd_cuda;
        object_rgbd_cuda.Upload(object_depth, color);

        open3d::cuda::TransformCuda camera_to_world_cuda;
        camera_to_world_cuda.FromEigen(camera_pose);

        open3d::cuda::ImageCuda<uchar, 1> mask_cuda;
        mask_cuda.Upload(instance_image.maskb_);

        volume_.Integrate(object_rgbd_cuda, intrinsic_cuda_, camera_to_world_cuda, mask_cuda);

#ifdef OSLAM_DEBUG_VIS
        if (frame.timestamp_ % 100 == 0)
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

}  // namespace oslam
