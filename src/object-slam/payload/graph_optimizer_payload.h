/******************************************************************************
* File:             graph_optimizer_payload.h
*
* Author:           Akash Sharma
* Created:          06/16/20
* Description:      Payload structure for graph optimizer module
*****************************************************************************/
#ifndef OSLAM_GRAPH_OPTIMIZER_PAYLOAD_H
#define OSLAM_GRAPH_OPTIMIZER_PAYLOAD_H

#include "utils/pipeline_payload.h"
namespace oslam {

    /*! \class OptimizerInputPayload
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class OptimizerInputPayload
    {
    public:
        OptimizerInputPayload(const Timestamp& r_timestamp_kf);
        virtual ~OptimizerInputPayload();

    protected:
        const Timestamp m_timestamp;
    };
}
#endif /* ifndef OSLAM_GRAPH_OPTIMIZER_PAYLOAD_H */
