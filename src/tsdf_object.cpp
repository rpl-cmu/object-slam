/******************************************************************************
 * File:             object.hpp
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      TSDFObject implementation
 *****************************************************************************/
#include "tsdf_object.h"
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <memory>
#include <tuple>
#include <vector>

namespace oslam {

TSDFObject::TSDFObject(unsigned int label,
  double score,
  open3d::camera::PinholeCameraIntrinsic r_intrinsic,
  double volume_size,
  double resolution)
  : m_intrinsic(r_intrinsic),
    m_object_volume(volume_size / resolution, 0.04, open3d::integration::TSDFVolumeColorType::RGB8),
    m_volume_size(volume_size), m_resolution(resolution), m_label(label), m_score(score)
{
    /* using namespace open3d::geometry; */
    /* auto bounding_box = */
    /*   std::make_shared<open3d::geometry::OrientedBoundingBox>(mp_pcd->GetOrientedBoundingBox()); */

    /* open3d::visualization::DrawGeometries( */
    /*   { mp_pcd, bounding_box }, fmt::format("TSDFObject {}", m_label), 1600, 900); */
}

}// namespace oslam
