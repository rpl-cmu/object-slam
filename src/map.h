/******************************************************************************
 * File:             map.h
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include "frame.h"
#include "tsdf_object.h"
#include "utils/macros.h"
namespace oslam
{
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
    static GlobalMap &get_instance()
    {
      static GlobalMap global_map;
      return global_map;
    }
    virtual ~GlobalMap() { mv_objects.clear(); }

    void add_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose);

    void add_object(const Frame &r_frame, const cv::Mat &r_mask, unsigned int label, double score,
                    const Eigen::Matrix4d &r_camera_pose);

    void integrate_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose);

    void raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                            open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose);

   private:
    explicit GlobalMap() = default;

    //! Vector of TSDF objects that are part of the map
    std::vector<TSDFObject::UniquePtr> mv_objects;

    std::mutex m_map_mutex;
  };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
