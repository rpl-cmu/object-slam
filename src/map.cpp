/******************************************************************************
 * File:             map.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/01/20
 * Description:      Global map containing objects
 *****************************************************************************/
#include "map.h"

#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Open3D/Camera/PinholeCameraIntrinsic.h>
/* #include <gtsam/geometry/Pose3.h> */
/* #include <gtsam/inference/Symbol.h> */
/* #include <gtsam/slam/BetweenFactor.h> */
/* #include <gtsam/slam/PriorFactor.h> */
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <vector>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    bool Map::addObject(TSDFObject::Ptr object, bool is_active_bg)
    {
        auto success = id_to_object_.insert(std::make_pair(object->id_, object));
        if (is_active_bg)
        {
            active_bg_id_ = object->id_;
        }
        return success.second;
    }
    TSDFObject::Ptr Map::getObject(const ObjectId &id)
    {
        IdToObjectMap::const_iterator it = id_to_object_.find(id);
        if (it == id_to_object_.end())
        {
            spdlog::error("Fatal: could not find object in the map");
            return nullptr;
        }
        return it->second;
    }

    TSDFObject::Ptr Map::getBackground() { return getObject(active_bg_id_); }

}  // namespace oslam
