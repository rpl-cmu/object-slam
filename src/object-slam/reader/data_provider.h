/******************************************************************************
 * File:             data_provider.h
 *
 * Author:           Akash Sharma
 * Created:          05/15/20
 * Description:      Data Provider module header
 *****************************************************************************/
#ifndef OSLAM_DATA_PROVIDER_H
#define OSLAM_DATA_PROVIDER_H

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"
#include "object-slam/utils/thread_safe_queue.h"

#include "object-slam/struct/frame.h"

namespace oslam
{
    /*! \class DataProvider
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class DataProvider : public SIMOPipelineModule<Frame, Frame>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(DataProvider);
        OSLAM_DELETE_COPY_CONSTRUCTORS(DataProvider);

        using SIMO       = SIMOPipelineModule<Frame, Frame>;
        using InputQueue = ThreadsafeQueue<typename SIMO::InputUniquePtr>;

        explicit DataProvider(InputQueue *p_input_queue);
        virtual ~DataProvider() = default;

        virtual OutputUniquePtr run_once(Frame::UniquePtr input);
    };
}  // namespace oslam
#endif /* ifndef OSLAM_DATA_PROVIDER_H */
