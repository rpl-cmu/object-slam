/******************************************************************************
 * File:             frame.h
 *
 * Author:
 * Created:          04/07/20
 * Description:      Frame header: Logical RGBD Frame object
 *****************************************************************************/
#ifndef OSLAM_FRAME_H
#define OSLAM_FRAME_H

#include <Eigen/Eigen>
#include <Open3D/Geometry/RGBDImage.h>
#include <Open3D/Open3D.h>
#include <memory>

namespace oslam {

struct RGBDdata;

struct Odometry
{
    Eigen::Matrix4d transform;
    Eigen::Matrix6d information;

    Odometry(const Eigen::Matrix4d &r_transform = Eigen::Matrix4d::Identity(),
      const Eigen::Matrix6d &r_information = Eigen::Matrix6d::Identity())
      : transform(r_transform), information(r_information)
    {}
};


class Frame
{
  public:
    Frame(std::size_t index,
      const RGBDdata &r_data,
      open3d::camera::PinholeCameraIntrinsic intrinsic);
    virtual ~Frame();

    std::shared_ptr<Odometry> odometry(std::shared_ptr<Frame> p_target_frame,
      const Eigen::Matrix4d &r_init_transformation = Eigen::Matrix4d::Identity());

    void visualize();

    inline std::shared_ptr<open3d::geometry::RGBDImage> get_rgbd(void) const { return mp_rgbd; }
    inline Eigen::Matrix4d get_pose(void) const { return m_pose; }
    inline void set_pose(Eigen::Matrix4d pose) { m_pose = std::move(pose); }

  private:
    std::size_t m_frame_id;
    open3d::geometry::Image m_color;
    open3d::geometry::Image m_depth;
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    std::shared_ptr<open3d::geometry::RGBDImage> mp_rgbd;
    std::shared_ptr<open3d::geometry::PointCloud> mp_pcd;
    Eigen::Matrix4d m_pose;
};
}// namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
