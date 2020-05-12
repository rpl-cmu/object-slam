/******************************************************************************
* File:             map.h
*
* Author:           Akash Sharma
* Created:          05/01/20
* Description:      Global map containing objects
*****************************************************************************/
#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include "object.h"
namespace oslam {

    class GlobalMap
    {
    public:
        GlobalMap ();
        GlobalMap (std::shared_ptr<oslam::Object> p_background_object);
        virtual ~GlobalMap () = default;

    private:
        /* data */
        std::vector<std::shared_ptr<oslam::Object>> mv_objects;
    };
}
#endif /* ifndef OSLAM_MAP_H */
