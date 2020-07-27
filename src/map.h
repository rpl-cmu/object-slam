/******************************************************************************
 * File:             map.h
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include <Cuda/Geometry/ImageCuda.h>

#include <condition_variable>
#include <map>
/* #include <gtsam/nonlinear/NonlinearFactorGraph.h> */

#include "frame.h"
#include "instance_image.h"
#include "tsdf_object.h"
#include "utils/macros.h"
#include "utils/thread_sync_var.h"
#include "utils/types.h"

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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using IdToObjectMap = std::unordered_map<ObjectId, TSDFObject::Ptr>;

        explicit Map() = default;
        virtual ~Map() { id_to_object_.clear(); }

        bool addObject(TSDFObject::Ptr object, bool is_active_bg = false);
        bool removeObject(const ObjectId& id);

        TSDFObject::Ptr getObject(const ObjectId &id);
        TSDFObject::Ptr getBackground();


        //! Map is a hashtable of different objects
        IdToObjectMap id_to_object_;
        ObjectId active_bg_id_;

        private:
        std::mutex mutex_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAP_H */
