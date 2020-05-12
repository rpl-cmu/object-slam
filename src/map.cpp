/******************************************************************************
* File:             map.cpp
*
* Author:           Akash Sharma
* Created:          05/01/20
* Description:      Global map containing objects
*****************************************************************************/
#include "map.h"
#include <memory>

namespace oslam {

    //TODO: Default constructor
    GlobalMap::GlobalMap()
    {
    }

    GlobalMap::GlobalMap(std::shared_ptr<open3d::integration::ScalableTSDFVolume> p_background_object)
    {
        mv_objects.push_back(p_background_object);
    }
}
