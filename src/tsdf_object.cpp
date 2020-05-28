/******************************************************************************
 * File:             object.hpp
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      TSDFObject implementation
 *****************************************************************************/
#include "tsdf_object.h"
#include <Cuda/Geometry/ImageCuda.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <spdlog/spdlog.h>
#include <tuple>
#include <vector>

#include <Cuda/Common/TransformCuda.h>
#include <Cuda/Geometry/PointCloudCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>

namespace oslam {

TSDFObject::TSDFObject(open3d::geometry::Image &r_color,
  open3d::geometry::Image &r_object_depth,
  unsigned int label,
  double score,
  open3d::cuda::PinholeCameraIntrinsicCuda &r_intrinsic,
  Eigen::Matrix4d camera_pose,
  int resolution)
  : m_resolution(resolution), m_label(label), m_score(score), m_pose(camera_pose), mc_intrinsic(r_intrinsic)
{
    open3d::cuda::RGBDImageCuda object_rgbd;
    object_rgbd.Upload(r_object_depth, r_color);

    open3d::cuda::PointCloudCuda object_point_cloud(open3d::cuda::VertexWithColor, r_color.width_ * r_color.height_);
    object_point_cloud.Build(object_rgbd, mc_intrinsic);

    /* //! Translate the object pose to the object point cloud center */
    /* m_pose.block(0, 3, 3, 3) = object_center; */

    //! Appropriately size the voxel_length of volume for appropriate resolution of the object
    Eigen::Vector3d object_max = object_point_cloud.GetMaxBound();
    Eigen::Vector3d object_min = object_point_cloud.GetMinBound();
    float voxel_length = 0.9 * (object_max - object_min).maxCoeff() / resolution;

    spdlog::debug("Volume pose\n {}", m_pose);
    spdlog::debug("Voxel length {}", voxel_length);
    open3d::cuda::TransformCuda c_object_pose;
    c_object_pose.FromEigen(m_pose);
    //TODO: Debug this
    mpc_object_volume.emplace(M_SUBVOLUME_RES, voxel_length, 4 * voxel_length, c_object_pose);
}

void TSDFObject::integrate(open3d::geometry::Image &r_color,
  open3d::geometry::Image &r_depth,
  cv::Mat &r_mask,
  open3d::camera::PinholeCameraIntrinsic intrinsics,
  Eigen::Matrix4d camera_pose)
{
    std::scoped_lock<std::mutex> integration_lock(m_object_mutex);

    //! TODO Do we integrate over the entire volume or just the mask?
    //! Mask out the depth image using the mask binary matrix
    cv::Mat object_dep(r_depth.height_, r_depth.width_, CV_16UC1, r_depth.data_.data());
    cv::Mat object_depth = object_dep.clone();
    object_depth.setTo(0, ~r_mask);

    open3d::geometry::Image o3d_object_depth;
    o3d_object_depth.Prepare(
      r_depth.width_, r_depth.height_, r_depth.num_of_channels_, r_depth.bytes_per_channel_);
    o3d_object_depth.data_.assign(object_depth.datastart, object_depth.dataend);

    /* open3d::geometry::RGBDImage object_rgbd(r_color, o3d_object_depth); */
    /* std::shared_ptr<open3d::geometry::PointCloud> p_pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(object_rgbd, intrinsics); */

    /* open3d::visualization::DrawGeometries({ p_pcd }, "Object pointcloud"); */
    open3d::cuda::RGBDImageCuda c_object_rgbd(r_color.width_, r_color.height_, 3.0f, 1000.0f);
    c_object_rgbd.Upload(o3d_object_depth, r_color);

    /* open3d::cuda::PointCloudCuda c_pcd(open3d::cuda::VertexWithColor, r_color.width_ * r_color.height_); */
    /* c_pcd.Build(c_object_rgbd, mc_intrinsic); */

    /* std::shared_ptr<open3d::geometry::PointCloud> download_pcl = c_pcd.Download(); */

    /* open3d::visualization::DrawGeometries({ download_pcl }, "Downloaded PCL"); */
    //! T_camera_2_object =  T_camera_2_world.inverse * T_object_2_world
    /* Eigen::Matrix4d T_object_2_camera = camera_pose.inverse() * m_pose; */
    /* spdlog::debug("Current background pose\n {}", camera_pose); */
    open3d::cuda::TransformCuda c_object_2_world;
    c_object_2_world.FromEigen(camera_pose);

    mpc_object_volume->Integrate(c_object_rgbd, mc_intrinsic,c_object_2_world);

    /* mpc_object_volume->GetAllSubvolumes(); */

    /* open3d::cuda::ScalableMeshVolumeCuda mesher( */
    /*         open3d::cuda::VertexWithNormalAndColor, 16, */
    /*         mpc_object_volume->active_subvolume_entry_array_.size(), 2000000, 4000000); */

    /* mesher.MarchingCubes(*mpc_object_volume); */

    /* auto mesh = mesher.mesh().Download(); */

    /* spdlog::debug("Showing mesh"); */
    /* open3d::visualization::DrawGeometries({ mesh }, "Mesh after integration"); */
    /* spdlog::debug("Done showing mesh"); */
}

void TSDFObject::raycast(open3d::cuda::ImageCuda<float, 3> &vertex,
  open3d::cuda::ImageCuda<float, 3> &normal,
  open3d::cuda::ImageCuda<uchar, 3> &color,
  open3d::cuda::PinholeCameraIntrinsicCuda intrinsics,
  Eigen::Matrix4d camera_pose)
{
    //! TODO: Need to translate the poses
    open3d::cuda::TransformCuda c_camera_2_object;
    c_camera_2_object.FromEigen(camera_pose);
    mpc_object_volume->RayCasting(vertex, normal, color, intrinsics, c_camera_2_object);
}

}// namespace oslam
