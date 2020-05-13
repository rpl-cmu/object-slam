/******************************************************************************
 * File:             object.h
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      TSDFObject class
 *****************************************************************************/
#ifndef OSLAM_OBJECT_H
#define OSLAM_OBJECT_H

#include <Open3D/Open3D.h>
#include <memory>

namespace oslam {

class TSDFObject
{
  public:
    TSDFObject(unsigned int label,
      double score,
      open3d::camera::PinholeCameraIntrinsic r_intrinsic,
      double volume_size = 256.0,
      double resolution = 4.0);
    virtual ~TSDFObject() = default;

  private:
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    open3d::integration::ScalableTSDFVolume m_object_volume;

    double m_volume_size;
    double m_resolution;
    unsigned int m_label;
    double m_score;

    // list of frame IDs where observed
    std::vector<std::size_t> m_observations;
    Eigen::Matrix4d m_pose;
};
}// namespace oslam
#endif /* ifndef OSLAM_OBJECT_H */
