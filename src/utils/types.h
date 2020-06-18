/******************************************************************************
* File:             types.h
*
* Author:           Akash Sharma
* Created:          06/16/20
* Description:      Utility types for Object SLAM
*****************************************************************************/
#ifndef OSLAM_TYPES_H
#define OSLAM_TYPES_H

#include <cstdint>
namespace oslam {

using Timestamp = std::uint64_t;

using FrameId = std::uint64_t;

using ObjectInstanceId    = unsigned int;
using ObjectLabelId = unsigned int;



}
#endif /* ifndef OSLAM_TYPES_H */
