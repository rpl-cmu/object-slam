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
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <vector>

#include "object-slam/utils/utils.h"

#include "object-slam/struct/instance_image.h"

namespace oslam
{
    bool Map::addObject(TSDFObject::Ptr object, bool is_active_bg)
    {
        std::scoped_lock<std::mutex> lock_add_object(mutex_);
        auto success = id_to_object_.insert(std::make_pair(object->id_, object));
        if (is_active_bg)
        {
            active_bg_id_ = object->id_;
        }
        spdlog::debug("Added new entry: {} to the map, new size: {}", object->id_, id_to_object_.size());
        return success.second;
    }

    bool Map::removeObject(const ObjectId& id)
    {
        std::scoped_lock<std::mutex> lock_remove_object(mutex_);
        auto success = id_to_object_.erase(id);
        if(success == 0U)
        {
            spdlog::debug("Failed to remove object: {} from the map, {} does not exist", id);
            return false;
        }
        return true;
    }
    TSDFObject::Ptr Map::getObject(const ObjectId &id)
    {
        std::scoped_lock<std::mutex> lock_get_object(mutex_);
        auto it = id_to_object_.find(id);
        if (it == id_to_object_.end())
        {
            spdlog::error("Fatal: could not find object in the map");
            return nullptr;
        }
        return it->second;
    }

    TSDFObject::Ptr Map::getBackground() { return getObject(active_bg_id_); }

}  // namespace oslam
