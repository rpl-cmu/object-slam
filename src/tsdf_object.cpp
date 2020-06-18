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
#include <opencv2/core/mat.hpp>
#include <tuple>
#include <vector>

namespace oslam
{
  TSDFObject::TSDFObject(const Frame &r_frame, const cv::Mat &r_mask, unsigned int label, double score,
                         const Eigen::Matrix4d &r_camera_pose, int resolution)
      : m_resolution(resolution),
        m_label(label),
        m_score(score),
        m_pose(r_camera_pose),
        m_intrinsic(r_frame.get_intrinsics()),
        mc_intrinsic(m_intrinsic)
  {
    auto color            = r_frame.get_color();
    auto o3d_object_depth = get_masked_depth(r_frame, r_mask);

    open3d::cuda::RGBDImageCuda object_rgbd;
    object_rgbd.Upload(o3d_object_depth, color);
    open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexWithColor, color.width_ * color.height_);
    object_point_cloud.Build(object_rgbd, mc_intrinsic);

    //! Translate the object pose to point cloud center
    //! T_wo = T_wc * T_co
    m_pose.block<3, 1>(0, 3) = object_point_cloud.GetCenter();
    spdlog::info("Object pose: {}", m_pose);
    m_pose                   = r_camera_pose * m_pose;

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
    mpc_object_volume.emplace(M_SUBVOLUME_RES, voxel_length, 4 * voxel_length, c_object_pose, 500, 8000);
  }

  open3d::geometry::Image TSDFObject::get_masked_depth(const Frame &r_frame, const cv::Mat &r_mask)
  {
    auto depth = r_frame.get_depth();
    cv::Mat object_dep(depth.height_, depth.width_, CV_16UC1, depth.data_.data());
    cv::Mat object_depth = object_dep.clone();
    object_depth.setTo(0, ~r_mask);

    open3d::geometry::Image o3d_object_depth;
    o3d_object_depth.Prepare(depth.width_, depth.height_, depth.num_of_channels_, depth.bytes_per_channel_);
    o3d_object_depth.data_.assign(object_depth.datastart, object_depth.dataend);
    return o3d_object_depth;
  }

  void TSDFObject::integrate(const Frame &r_frame, const cv::Mat &r_mask, const Eigen::Matrix4d &r_camera_pose)
  {
    std::scoped_lock<std::mutex> integration_lock(m_object_mutex);

    auto color = r_frame.get_color();
    //! TODO Do we integrate over the entire volume or just the mask?
    //! Mask out the depth image using the mask binary matrix
    auto o3d_object_depth = get_masked_depth(r_frame, r_mask);

    open3d::cuda::RGBDImageCuda c_object_rgbd(color.width_, color.height_, 3.0f, 1000.0f);
    c_object_rgbd.Upload(o3d_object_depth, color);

    open3d::cuda::TransformCuda c_camera_2_world;
    c_camera_2_world.FromEigen(r_camera_pose);

    mpc_object_volume->Integrate(c_object_rgbd, mc_intrinsic, c_camera_2_world);

    /* if(r_frame.m_timestamp % 50 == 0) */
    /* { */
    /*     mpc_object_volume->GetAllSubvolumes(); */
    /*     open3d::cuda::ScalableMeshVolumeCuda mesher(open3d::cuda::VertexWithNormalAndColor, 16, */
    /*                                                 mpc_object_volume->active_subvolume_entry_array_.size(), 2000000, 4000000); */
    /*     mesher.MarchingCubes(*mpc_object_volume); */
    /*     auto mesh = mesher.mesh().Download(); */
    /*     open3d::visualization::DrawGeometries({ mesh }, "Mesh after integration"); */
    /* } */
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
