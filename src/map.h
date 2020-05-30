/******************************************************************************
 * File:             map.h
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include "tsdf_object.h"
#include "utils/macros.h"
namespace oslam {

/*! \class GlobalMap
 *  \brief Brief class description
 *
 *  Detailed description
 */
class GlobalMap
{
  public:
    OSLAM_DELETE_COPY_CONSTRUCTORS(GlobalMap);
    //! Singleton Global map object
    static GlobalMap &get_instance(void)
    {
        static GlobalMap global_map;
        return global_map;
    }
    virtual ~GlobalMap() = default;


    void add_background(open3d::geometry::Image &r_color,
      open3d::geometry::Image &r_depth,
      open3d::cuda::PinholeCameraIntrinsicCuda &r_intrinsic,
      open3d::camera::PinholeCameraIntrinsic intrinsics,
      const Eigen::Matrix4d &r_camera_pose);

    void add_object(open3d::geometry::Image &r_color,
      open3d::geometry::Image &r_depth,
      cv::Mat &r_mask,
      unsigned int label,
      double score,
      open3d::cuda::PinholeCameraIntrinsicCuda &r_intrinsic,
      Eigen::Matrix4d &r_camera_pose);

    void integrate_background(open3d::geometry::Image &r_color,
      open3d::geometry::Image &r_depth,
      Eigen::Matrix4d camera_pose);

    void raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex,
      open3d::cuda::ImageCuda<float, 3> &normal,
      open3d::cuda::ImageCuda<uchar, 3> &color,
      open3d::cuda::PinholeCameraIntrinsicCuda intrinsic,
      Eigen::Matrix4d camera_pose);

  private:
    explicit GlobalMap(){};

    //! Vector of TSDF objects that are part of the map
    std::vector<TSDFObject::UniquePtr> mv_objects;

    std::mutex m_map_mutex;
};
}// namespace oslam
#endif /* ifndef OSLAM_MAP_H */
