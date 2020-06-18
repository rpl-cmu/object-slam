/******************************************************************************
 * File:             map.h
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include <map>

#include "frame.h"
#include "tsdf_object.h"
#include "utils/macros.h"
#include "utils/types.h"

namespace oslam
{
  /*! \class GlobalMap
   *  \brief Brief class description
   *
   *  Detailed description
   */
  class GlobalMap
  {
    using LabelToObjectsMap = std::multimap<ObjectLabelId, TSDFObject::UniquePtr>;

   public:
    OSLAM_DELETE_COPY_CONSTRUCTORS(GlobalMap);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Singleton Global map object
    static GlobalMap &get_instance()
    {
      static GlobalMap global_map;
      return global_map;
    }
    virtual ~GlobalMap() { m_label_to_objects.clear(); }

    void add_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose);

    void add_object(const Frame &r_frame, const cv::Mat &r_mask, unsigned int label, double score,
                    const Eigen::Matrix4d &r_camera_pose);

    void integrate_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose);
    void integrate_object(const Frame &r_frame, const cv::Mat &r_mask, unsigned int label, double score,
                          const Eigen::Matrix4d &r_camera_pose, const Eigen::Matrix4d &r_T_maskcamera_2_camera);

    void raycast_background(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                            open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose);

   private:
    explicit GlobalMap() = default;

    constexpr static double SCORE_THRESHOLD = 0.6;
    //! Map contains a multimap of objects (multiple instances of an object label)
    LabelToObjectsMap m_label_to_objects;

    std::mutex m_map_mutex;
  };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
