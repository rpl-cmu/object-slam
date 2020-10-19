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

#include <cassert>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <tuple>
#include <vector>
#include <xtensor/xmath.hpp>
#include "object-slam/utils/utils.h"

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
        // Move object point cloud to F_{W}
        object_point_cloud.Transform(camera_pose);

        object_max_pt_ = Eigen::Affine3d(pose_) * object_point_cloud.GetMaxBound();
        object_min_pt_ = Eigen::Affine3d(pose_) * object_point_cloud.GetMinBound();
        // Translate the object pose to point cloud center
        // T_wo = T_wc * T_co
        Eigen::Matrix4d T_camera_to_object   = Eigen::Matrix4d::Identity();
        T_camera_to_object.block<3, 1>(0, 3) = object_min_pt_;
        pose_                                = camera_pose * T_camera_to_object.inverse();

        // Appropriately size the voxel_length of volume for appropriate resolution of the object
        voxel_length_ = VOLUME_SIZE_SCALE * float((object_max_pt_ - object_min_pt_).maxCoeff() / resolution);

        spdlog::debug("Volume pose\n {}", pose_);
        spdlog::debug("Voxel length: {}", voxel_length_);

        // TODO: Check if this should be camera pose
        cuda::TransformCuda object_pose_cuda;
        object_pose_cuda.FromEigen(pose_);

        // truncation distance = 4 * voxel length
        // Allocate lower memory since we will create multiple TSDF objects
        if (instance_image.label_ == 0)
        {
            BUCKET_COUNT_   = 12000;
            VALUE_CAPACITY_ = 12000;
            spdlog::debug("Created new background instance");
        }
        else
        {
            spdlog::debug("Created new object instance");
        }
        volume_ = cuda::ScalableTSDFVolumeCuda(SUBVOLUME_RES,
                                               voxel_length_,
                                               TSDF_TRUNCATION_SCALE * voxel_length_,
                                               object_pose_cuda,
                                               BUCKET_COUNT_,
                                               VALUE_CAPACITY_);
        //! Empty until the first time the object goes out of camera frustum
        volume_cpu_ = std::nullopt;
    }

    void TSDFObject::integrate(const Frame &frame, const InstanceImage &instance_image, const Eigen::Matrix4d &camera_pose)
    {
        std::scoped_lock<std::mutex> lock_integration(mutex_);
        spdlog::trace("Entered integrate for object: {}", id_);
        if (volume_.device_ == nullptr)
        {
            spdlog::error("Volume downloaded into CPU, cannot integrate");
            assert(false);
        }
        auto color        = frame.color_;
        auto object_depth = frame.depth_.clone();
        object_depth.setTo(0, ~instance_image.bbox_mask_);


        if (!isBackground())
        {
            auto difference   = instance_image_.feature_ - instance_image.feature_;
            float match_score = cv::norm(difference, cv::NORM_L1);

            if (match_score < 150)
            {
                instance_image_.feature_ = instance_image.feature_;
                spdlog::debug("Updated feature map for the object");
            }
        }

        /* auto match_score = xt::norm_l1(difference); */

        /* spdlog::info("Matching score between object: {}", match_score); */
        open3d::cuda::RGBDImageCuda object_rgbd_cuda;
        object_rgbd_cuda.Upload(object_depth, color);

        /* open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexRaw, frame.width_ * frame.height_); */
        /* object_point_cloud.Build(object_rgbd_cuda, intrinsic_cuda_); */

        /* auto cpu_pcl = object_point_cloud.Download(); */
        /* open3d::visualization::DrawGeometries({cpu_pcl}, "Integrating object cloud"); */
        open3d::cuda::TransformCuda camera_to_object_cuda;
        /* Eigen::Matrix4d camera_to_object = pose_.inverse() * camera_pose; */
        camera_to_object_cuda.FromEigen(camera_pose);

        open3d::cuda::ImageCuda<uchar, 1> mask_cuda;
        mask_cuda.Upload(instance_image.maskb_);

        volume_.Integrate(
            object_rgbd_cuda, intrinsic_cuda_, camera_to_object_cuda, static_cast<int>(frame.timestamp_), mask_cuda);

#ifdef OSLAM_DEBUG_VIS
        if (frame.timestamp_ % 100 == 0)
        {
            volume_.GetAllSubvolumes();
            open3d::cuda::ScalableMeshVolumeCuda mesher(
                open3d::cuda::VertexWithColor, 16, volume_.active_subvolume_entry_array_.size(), 200000, 400000);
            mesher.MarchingCubes(volume_);
            auto mesh = mesher.mesh().Download();
            auto aabb =
                std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(volume_.GetMinBound(), volume_.GetMaxBound());
            open3d::visualization::DrawGeometries({ mesh, aabb }, "Mesh after integration");
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
        if (volume_.device_ == nullptr)
        {
            spdlog::error("Volume downloaded into CPU, cannot raycast");
        }
        open3d::cuda::TransformCuda camera_to_object_cuda;
        /* Eigen::Matrix4d camera_to_object = pose_.inverse() * camera_pose; */
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

    void TSDFObject::downloadVolumeToCPU()
    {
        assert(volume_cpu_.has_value() && !volume_.device_);
        if (volume_.device_)
        {
            volume_cpu_ = std::make_optional(volume_.DownloadVolumes());
            volume_.Release();
        }
    }

    void TSDFObject::uploadVolumeToGPU()
    {
        assert(volume_cpu_.has_value());
        if (volume_.device_)
        {
            spdlog::warn("Object already on GPU, not uploading");
            return;
        }
        auto keys    = volume_cpu_->first;
        auto volumes = volume_cpu_->second;

        spdlog::info("Length of keys: {}, length of volumes: {}", keys.size(), volumes.size());
        cuda::TransformCuda object_pose_cuda;
        object_pose_cuda.FromEigen(pose_);

        cuda::ScalableTSDFVolumeCuda new_volume = cuda::ScalableTSDFVolumeCuda(SUBVOLUME_RES,
                                                                               voxel_length_,
                                                                               TSDF_TRUNCATION_SCALE * voxel_length_,
                                                                               object_pose_cuda,
                                                                               BUCKET_COUNT_,
                                                                               VALUE_CAPACITY_);

        new_volume.UploadVolumes(keys, volumes);
        std::swap(volume_, new_volume);
        volume_cpu_.reset();
    }

    Eigen::Vector3d TSDFObject::getMinBound()
    {
        if (volume_.device_ != nullptr)
        {
            object_min_pt_ = volume_.GetMinBound();
            return object_min_pt_;
        }
        return object_min_pt_;
    }

    Eigen::Vector3d TSDFObject::getMaxBound()
    {
        if (volume_.device_ != nullptr)
        {
            object_max_pt_ = volume_.GetMaxBound();
            return object_max_pt_;
        }
        return object_max_pt_;
    }
}  // namespace oslam
