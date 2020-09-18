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

#include "object-slam/utils/macros.h"
#include "object-slam/utils/thread_sync_var.h"

#include "object-slam/struct/frame.h"
#include "object-slam/struct/instance_image.h"

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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Constructor accepts masked depth image to size the object volume during
        //! allocation/initialization
        TSDFObject(const ObjectId &id,
                   const Frame &frame,
                   const InstanceImage &instance_image,
                   const Eigen::Matrix4d &camera_pose = Eigen::Matrix4d::Identity(),
                   int resolution                     = 64);

        virtual ~TSDFObject() = default;

        [[nodiscard]] bool isBackground() const { return (instance_image_.label_ == 0); }
        [[nodiscard]] unsigned int getLabel() const { return instance_image_.label_; }

        void integrate(const Frame &frame, const InstanceImage &instance_image, const Eigen::Matrix4d &camera_pose);

        void raycast(open3d::cuda::ImageCuda<float, 3> &vertex,
                     open3d::cuda::ImageCuda<float, 3> &normal,
                     open3d::cuda::ImageCuda<uchar, 3> &color,
                     const Eigen::Matrix4d &camera_pose);

        [[nodiscard]] Eigen::Matrix4d getPose() const { return pose_; }
        void setPose(const Eigen::Matrix4d &pose)
        {
            std::scoped_lock<std::mutex> lock_setpose(mutex_);
            pose_ = pose;
        }
        [[nodiscard]] cuda::PinholeCameraIntrinsicCuda getIntrinsicCuda() const { return intrinsic_cuda_; }

        [[nodiscard]] double getExistExpectation() const { return double(existence_) / double(existence_ + non_existence_); }

        [[nodiscard]] double getVisibilityRatio(Timestamp timestamp) const;

        const ObjectId id_;             //!< Const public object ID Cannot be modified
        std::hash<ObjectId> hash;  //!< Functor for obtaining hash from object ID

       private:
        constexpr static int SUBVOLUME_RES                = 16;  //!< Each subvolume unit has 16^3 voxels
        constexpr static float VOLUME_SIZE_SCALE          = 0.7F;
        constexpr static float TSDF_TRUNCATION_SCALE      = 5.0F;
        constexpr static int RETROSPECT_VISIBILITY_THRESH = 5;

        int resolution_;                //!< Resolution for the object volume (about 128^3 voxels)
        InstanceImage instance_image_;  //!< Object semantic information
        Eigen::Matrix4d pose_;          //!< Object pose w.r.t world frame T_o_w
        open3d::camera::PinholeCameraIntrinsic intrinsic_;
        open3d::cuda::PinholeCameraIntrinsicCuda intrinsic_cuda_;
        std::mutex mutex_;              //!< Protection against integration/raycasting from multiple threads
        open3d::cuda::ScalableTSDFVolumeCuda volume_;

        std::atomic_int existence_     = 1;
        std::atomic_int non_existence_ = 1;

        Eigen::Vector3d object_max_pt_;
        Eigen::Vector3d object_min_pt_;

        friend struct Map;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_OBJECT_H */
