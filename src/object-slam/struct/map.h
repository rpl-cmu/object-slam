/******************************************************************************
 * File:             map.h
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include <gtsam/nonlinear/Values.h>
#include <map>
#include <mutex>

#include "object-slam/utils/macros.h"
#include "object-slam/utils/types.h"

#include "object-slam/struct/frame.h"
#include "object-slam/struct/instance_image.h"
#include "object-slam/struct/tsdf_object.h"

namespace oslam
{
    /*! \class Map
     *  \brief A map representing a scene consists of a hashtable of object volumes, each with their own poses
     *  and hence forming a posegraph
     */
    struct Map
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Map);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Map);
        OSLAM_DELETE_MOVE_CONSTRUCTORS(Map);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using IdToObjectMap = std::unordered_map<ObjectId, TSDFObject::Ptr>;

        explicit Map() = default;
        virtual ~Map() = default;

        void addBackground(TSDFObject::UniquePtr bg);
        void integrateBackground(const Frame& frame,
                                 const InstanceImage& instance_image,
                                 const Eigen::Matrix4d& camera_pose);
        Render renderBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose);
        ObjectId getBackgroundId()
        {
            std::scoped_lock<std::mutex> lock_get_bg_id(mutex_);
            return background_volume_->id_;
        }
        double getBackgroundVisibilityRatio(const Timestamp& timestamp) const;

        bool addObject(TSDFObject::UniquePtr object);
        void integrateObject(const ObjectId& id,
                             const Frame& frame,
                             const InstanceImage& instance_image,
                             const Eigen::Matrix4d& camera_pose);
        Renders renderObjects(const Frame& frame, const Eigen::Matrix4d& camera_pose);
        std::uint64_t getObjectHash(const ObjectId& id) const;
        Eigen::Matrix4d getObjectPose(const ObjectId& id) const;

        size_t getNumObjects() const
        {
            std::scoped_lock<std::mutex> get_num_objects(mutex_);
            return id_to_object_.size();
        }

        std::vector<std::uint64_t> deleteBadObjects();

        void incrementExistence(const ObjectId& id);
        void incrementNonExistence(const ObjectId& id);

        void addCameraPose(const Eigen::Matrix4d& camera_pose);
        Eigen::Matrix4d getCameraPose(const Timestamp& camera_timestamp);

        PoseTrajectory getCameraTrajectory() const;

        void update(const gtsam::Values& values, const std::vector<Timestamp>& keyframe_timestamps);

       private:
        bool removeObject(const ObjectId& id);

        TSDFObject::UniquePtr background_volume_;
        PoseTrajectory camera_trajectory_;  //! Sequence of the camera poses traversed in the map
        IdToObjectMap id_to_object_;        //! Map is a hashtable of different objects
        mutable std::mutex mutex_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
