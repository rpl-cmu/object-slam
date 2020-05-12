/******************************************************************************
 * File:             object.hpp
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      Object implementation
 *****************************************************************************/
#include "object.h"
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <memory>
#include <tuple>
#include <vector>

namespace oslam {

Object::Object(std::shared_ptr<open3d::geometry::RGBDImage> p_object_rgbd,
  unsigned int label,
  double score,
  open3d::camera::PinholeCameraIntrinsic r_intrinsic)
  : m_intrinsic(r_intrinsic),
    m_object_volume(4 / 256.0, 0.04, open3d::integration::TSDFVolumeColorType::RGB8),
    m_label(label), m_score(score)
{
    using namespace open3d::geometry;
    mp_pcd = PointCloud::CreateFromRGBDImage(*p_object_rgbd, m_intrinsic);

    // TODO: Computationally intensive, remove maybe if not required
    /* std::shared_ptr<PointCloud> denoised_pcd; */
    /* std::vector<std::size_t> ind; */
    /* std::tie(denoised_pcd, ind) = mp_pcd->RemoveRadiusOutliers(50, 0.08); */

    /* denoised_pcd->PaintUniformColor({ 0.8, 0.8, 0.8 }); */

    auto bounding_box = std::make_shared<open3d::geometry::OrientedBoundingBox>(
      mp_pcd->GetOrientedBoundingBox());

    open3d::visualization::DrawGeometries(
      { mp_pcd, bounding_box }, fmt::format("Object {}", m_label), 1600, 900);
}

}// namespace oslam
