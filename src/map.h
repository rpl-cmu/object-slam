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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "frame.h"
#include "instance_image.h"
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
        using IdToObjectMap = std::unordered_map<ObjectId, TSDFObject::Ptr>;

       public:
        OSLAM_DELETE_COPY_CONSTRUCTORS(GlobalMap);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Singleton Global map object
        static GlobalMap &get_instance()
        {
            static GlobalMap global_map;
            return global_map;
        }
        virtual ~GlobalMap() { m_id_to_object.clear(); }

        std::pair<ObjectId, TSDFObject::Ptr> create_background(const Frame &r_frame, const Eigen::Matrix4d &r_camera_pose);
        std::pair<ObjectId, TSDFObject::Ptr> create_object(const Frame &r_frame, const InstanceImage &r_instance_image, const Eigen::Matrix4d &r_camera_pose);

        bool integrate_background(const Frame& r_frame, const Eigen::Matrix4d& r_camera_pose);
        bool integrate_object(const Frame &r_frame, const InstanceImage& r_instance_image,
                              const Eigen::Matrix4d &r_camera_pose);

        void raycast(const ObjectId& r_id, open3d::cuda::ImageCuda<float, 3> &vertex, open3d::cuda::ImageCuda<float, 3> &normal,
                                open3d::cuda::ImageCuda<uchar, 3> &color, const Eigen::Matrix4d &r_camera_pose);

        inline std::size_t size() const { return m_id_to_object.size(); }

        TSDFObject::ConstPtr get_object(const ObjectId& r_id) const;
       private:
        explicit GlobalMap() = default;

        constexpr static double SCORE_THRESHOLD = 0.5;
        constexpr static int MASK_AREA_THRESHOLD = 2500;

        //! Map is a hashtable of different objects
        IdToObjectMap m_id_to_object;

        TSDFObject::Ptr m_active_background;

        std::mutex m_map_mutex;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
