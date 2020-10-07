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
#include <mutex>
#include <Eigen/Eigen>
#include <gtsam/nonlinear/Values.h>

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
    using ObjectBoundingBoxes = std::unordered_map<ObjectId, std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
    using IdToObjectMesh      = std::unordered_map<ObjectId, std::shared_ptr<open3d::geometry::TriangleMesh>>;
    using Plane3d = Eigen::Hyperplane<double, 3>;
    using PointPlanes = std::vector<std::pair<Eigen::Hyperplane<double, 3>, Eigen::Vector3d>>;

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
        ObjectId getBackgroundId() const
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
        ObjectBoundingBoxes getAllObjectBoundingBoxes() const;

        IdToObjectMesh meshAllObjects() const;

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
        PointPlanes getCameraFrustumPlanes(const Eigen::Matrix4d& camera_pose) const;

        bool isObjectInFrustum(const TSDFObject::Ptr& object, const Eigen::Matrix4d& camera_pose);

        PoseTrajectory getCameraTrajectory() const;

        void update(const gtsam::Values& values, const std::vector<Timestamp>& keyframe_timestamps);

       private:
        static constexpr double MIN_DEPTH = 0.01;
        static constexpr double MAX_DEPTH = 4.0;
        bool removeObject(const ObjectId& id);
        void getObjectBoundingBox(const ObjectId& id, Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const;

        TSDFObject::UniquePtr background_volume_;
        PoseTrajectory camera_trajectory_;  //! Sequence of the camera poses traversed in the map
        IdToObjectMap id_to_object_;        //! Map is a hashtable of different objects
        mutable std::mutex mutex_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
