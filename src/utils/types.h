/******************************************************************************
 * File:             types.h
 *
 * Author:           Akash Sharma
 * Created:          06/16/20
 * Description:      Utility types for Object SLAM
 *****************************************************************************/
#ifndef OSLAM_TYPES_H
#define OSLAM_TYPES_H

#include <array>
#include <cstdint>
#include <functional>

#include <fmt/ostream.h>

namespace oslam
{
    using Timestamp = std::uint64_t;

    using FrameId = std::uint64_t;

    using BoundingBox = std::array<int, 4>;

    //! Hashable object id struct
    struct ObjectId
    {
        unsigned int label;
        oslam::Timestamp frame_created;
        oslam::BoundingBox bbx;

        ObjectId(unsigned int _label, unsigned int _frame_created, BoundingBox _bbx)
            : label(_label), frame_created(_frame_created), bbx(_bbx)
        {
        }
        ObjectId() : label(0), frame_created(0), bbx({ 0, 0, 0, 0 }) {}

        ~ObjectId() = default;

        bool operator==(const ObjectId& r_id) const
        {
            return label == r_id.label && frame_created == r_id.frame_created &&
                   (bbx[0] == r_id.bbx[0] && bbx[1] == r_id.bbx[1] && bbx[2] == r_id.bbx[2] && bbx[3] == r_id.bbx[3]);
        }

        friend std::ostream& operator<<(std::ostream& os, const ObjectId& r_id)
        {
            return os << "L:" << r_id.label << "-T:" << r_id.frame_created << "-BBX:{" << r_id.bbx[0] << "," << r_id.bbx[1] << "}";
        }
    };

}  // namespace oslam

namespace std
{
    template<>
    struct hash<oslam::ObjectId>
    {
        std::size_t operator()(const oslam::ObjectId& r_id) const noexcept
        {
            std::size_t seed                = 0;
            constexpr uint32_t GOLDEN_RATIO = 0x9e3779b9;
            seed ^= std::hash<unsigned int>()(r_id.label) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
            seed ^= std::hash<oslam::FrameId>()(r_id.frame_created) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(r_id.bbx[0]) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(r_id.bbx[1]) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);

            return seed;
        }
    };
}  // namespace std
#endif /* ifndef OSLAM_TYPES_H */
