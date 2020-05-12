/******************************************************************************
 * File:             object.h
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      Object class
 *****************************************************************************/
#ifndef OSLAM_OBJECT_H
#define OSLAM_OBJECT_H

#include <Open3D/Open3D.h>
#include <memory>

namespace oslam {

class Object
{
  public:
    Object(std::shared_ptr<open3d::geometry::RGBDImage> p_object_rgbd,
      unsigned int label,
      double score,
      open3d::camera::PinholeCameraIntrinsic r_intrinsic);
    virtual ~Object() = default;

  private:
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    open3d::integration::ScalableTSDFVolume m_object_volume;
    std::shared_ptr<open3d::geometry::PointCloud> mp_pcd;

    unsigned int m_label;
    double m_score;
    std::vector<std::size_t> m_observations;
    Eigen::Matrix4d m_pose;
};
}// namespace oslam
#endif /* ifndef OSLAM_OBJECT_H */
