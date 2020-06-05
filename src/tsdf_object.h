/******************************************************************************
 * File:             object.h
 *
 * Author:           Akash Sharma
 * Created:          04/29/20
 * Description:      TSDFObject class
 *****************************************************************************/
#ifndef OSLAM_OBJECT_H
#define OSLAM_OBJECT_H

#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Open3D/Open3D.h>

#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>

#include "frame.h"
#include "utils/macros.h"

namespace oslam
{
  /*! \class TSDFObject
   *  \brief Brief class description
   *
   *  Detailed description
   */
  class TSDFObject
  {
   public:
    OSLAM_POINTER_TYPEDEFS(TSDFObject);

    //! Constructor accepts masked depth image to size the object volume during
    //! allocation/initialization
    explicit TSDFObject(const Frame &r_frame, unsigned int label, double score,
                        const Eigen::Matrix4d &r_camera_pose = Eigen::Matrix4d::Identity(), int resolution = 64);

    virtual ~TSDFObject() = default;

    [[nodiscard]] bool is_background() const { return (m_label == 0); }

    void integrate(const Frame &r_frame, const cv::Mat &r_mask, const Eigen::Matrix4d &r_camera_pose);

    void raycast(open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                 open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose);

   private:
    constexpr static int M_SUBVOLUME_RES = 16;

    //! Resolution for the object volume
    int m_resolution;

    //! Object semantic information
    unsigned int m_label;
    double m_score;

    //! Object pose w.r.t world frame T_o_w
    Eigen::Matrix4d m_pose;

    //! Camera intrinsics required for raycasting or integration
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    open3d::cuda::PinholeCameraIntrinsicCuda mc_intrinsic;

    //! Object volume (Requires delayed construction)
    std::optional<open3d::cuda::ScalableTSDFVolumeCuda> mpc_object_volume;

    // list of frame IDs where observed
    /* std::vector<std::size_t> m_observations; */

    //! Protection against integration/raycasting from multiple threads
    std::mutex m_object_mutex;
  };
}  // namespace oslam
#endif /* ifndef OSLAM_OBJECT_H */
