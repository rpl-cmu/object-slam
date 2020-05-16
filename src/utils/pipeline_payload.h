/**
 * @file   PipelinePayload.h
 * @brief  Base class for the payloads shared between pipeline modules.
 * @author Antoni Rosinol
 */

#ifndef OSLAM_PIPELINE_PAYLOAD
#define OSLAM_PIPELINE_PAYLOAD

#include "macros.h"
#include <Eigen/Eigen>


namespace oslam {
typedef std::uint64_t Timestamp;

struct PipelinePayload
{
    OSLAM_POINTER_TYPEDEFS(PipelinePayload);
    OSLAM_DELETE_COPY_CONSTRUCTORS(PipelinePayload);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit PipelinePayload(const Timestamp &timestamp)
    : m_timestamp(timestamp) {};
    virtual ~PipelinePayload() = default;

    // Untouchable timestamp of the payload.
    const Timestamp m_timestamp;
};

/**
 * @brief The NullPipelinePayload is an empty payload, used for those modules
 * that do not return a payload, such as the display module, which only
 * displays images and returns nothing.
 */
struct NullPipelinePayload : public PipelinePayload
{
    OSLAM_POINTER_TYPEDEFS(NullPipelinePayload);
    OSLAM_DELETE_COPY_CONSTRUCTORS(NullPipelinePayload);
    explicit NullPipelinePayload() : PipelinePayload(Timestamp()) {}
    virtual ~NullPipelinePayload() = default;
};

}// namespace oslam

#endif /* ifndef OSLAM_PIPELINE_PAYLOAD */
