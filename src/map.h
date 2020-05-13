/******************************************************************************
* File:             map.h
*
* Author:           Akash Sharma
* Created:          05/01/20
* Description:      Global map containing objects
*****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include "tsdf_object.h"
namespace oslam {

    class GlobalMap
    {
    public:
        explicit GlobalMap() = default;
        virtual ~GlobalMap () = default;

        void add_object_volume(std::shared_ptr<oslam::TSDFObject> p_object);
    private:
        /* data */
        std::vector<std::shared_ptr<oslam::TSDFObject>> mv_objects;
    };
}
#endif /* ifndef OSLAM_MAP_H */
