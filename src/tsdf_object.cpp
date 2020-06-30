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
    TSDFObject::TSDFObject(const Frame &r_frame, const InstanceImage &r_instance_image, const Eigen::Matrix4d &r_camera_pose,
                           int resolution)
        : m_resolution(resolution),
          m_instance_image(r_instance_image),
          m_pose(r_camera_pose),
          m_intrinsic(r_frame.m_intrinsic),
          mc_intrinsic(m_intrinsic)
    {
        auto color        = r_frame.m_color;
        auto object_depth = r_frame.m_depth.clone();
        object_depth.setTo(0, ~r_instance_image.m_bbox_mask);

        //! Construct Object RGBD to obtain center and estimate object size
        open3d::cuda::RGBDImageCuda object_rgbd;
        object_rgbd.Upload(object_depth, color);
        open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexWithColor, r_frame.m_width * r_frame.m_height);
        object_point_cloud.Build(object_rgbd, mc_intrinsic);

        //! Translate the object pose to point cloud center
        //! T_wo = T_wc * T_co
        m_pose.block<3, 1>(0, 3) = object_point_cloud.GetCenter();
        m_pose = r_camera_pose * m_pose;

        //! Appropriately size the voxel_length of volume for appropriate resolution of the object
        Eigen::Vector3d object_max = object_point_cloud.GetMaxBound();
        Eigen::Vector3d object_min = object_point_cloud.GetMinBound();
        double voxel_length        = 0.9 * (object_max - object_min).maxCoeff() / resolution;

        spdlog::debug("Volume pose\n {}", m_pose);
        spdlog::debug("Voxel length {}", voxel_length);

        open3d::cuda::TransformCuda c_object_pose;
        c_object_pose.FromEigen(r_camera_pose);

        //! Subvolume resolution is always 16, truncation distance = 4 * voxel length
        //! Allocate lower memory since we will create multiple TSDF objects
        if(r_instance_image.m_label == 0)
            mpc_object_volume.emplace(M_SUBVOLUME_RES, voxel_length, 5 * voxel_length, c_object_pose, 1000, 4000);
        else
            mpc_object_volume.emplace(M_SUBVOLUME_RES, voxel_length, 5 * voxel_length, c_object_pose, 1000, 2000);
    }

    void TSDFObject::integrate(const Frame &r_frame, const InstanceImage &r_instance_image,
                               const Eigen::Matrix4d &r_camera_pose)
    {
        std::scoped_lock<std::mutex> integration_lock(m_object_mutex);

        auto color        = r_frame.m_color;
        auto object_depth = r_frame.m_depth.clone();  // TODO: How to avoid cloning here?
        object_depth.setTo(0, ~r_instance_image.m_bbox_mask);

        cv::Mat cvt8;
        cv::convertScaleAbs(object_depth, cvt8, 0.25);

        open3d::cuda::RGBDImageCuda c_object_rgbd;
        c_object_rgbd.Upload(object_depth, color);

        /* open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexWithColor, r_frame.m_width * r_frame.m_height); */
        /* object_point_cloud.Build(c_object_rgbd, mc_intrinsic); */

        /* auto pcd = object_point_cloud.Download(); */

        /* open3d::visualization::DrawGeometries({ pcd }, "Object point cloud"); */


        open3d::cuda::TransformCuda c_camera_2_world;
        c_camera_2_world.FromEigen(r_camera_pose);

        open3d::cuda::ImageCuda<uchar, 1> c_mask;
        c_mask.Upload(r_instance_image.m_maskb);

        mpc_object_volume->Integrate(c_object_rgbd, mc_intrinsic, c_camera_2_world, c_mask);
        if(r_frame.m_timestamp % 100 == 0)
        {
            mpc_object_volume->GetAllSubvolumes();
            open3d::cuda::ScalableMeshVolumeCuda mesher(open3d::cuda::VertexWithNormalAndColor, 16,
                                                        mpc_object_volume->active_subvolume_entry_array_.size(), 20000, 40000);
            mesher.MarchingCubes(*mpc_object_volume);
            auto mesh = mesher.mesh().Download();
            open3d::visualization::DrawGeometries({ mesh }, "Mesh after integration");
            mesher.Release();
        }
    }

    void TSDFObject::raycast(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                             open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose)
    {
        //! TODO: Need to translate the poses
        open3d::cuda::TransformCuda c_camera_2_object;
        c_camera_2_object.FromEigen(r_camera_pose);
        mpc_object_volume->RayCasting(vertex, normal, color, mc_intrinsic, c_camera_2_object);
    }

}  // namespace oslam
