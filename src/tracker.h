/******************************************************************************
 * File:             tracker.h
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker Thread
 *****************************************************************************/
#ifndef OSLAM_TRACKER_H
#define OSLAM_TRACKER_H

#include <memory>
#include <mutex>
#include <thread>

#include "utils/macros.h"
#include "utils/pipeline_module.h"
#include "utils/thread_safe_queue.h"

#include "tracker_payload.h"



namespace oslam {

/*! \class Tracker
 *  \brief Brief class description
 *
 *  Detailed description
 */
class Tracker : public MISOPipelineModule<TrackerPayload, NullPipelinePayload>
{
public:
    OSLAM_POINTER_TYPEDEFS(Tracker);
    OSLAM_DELETE_COPY_CONSTRUCTORS(Tracker);

    using MISO = MISOPipelineModule<TrackerPayload, NullPipelinePayload>;
    using MaskedImageQueue = ThreadsafeQueue<MaskedImage::UniquePtr>;

    explicit Tracker(MaskedImageQueue* p_masked_image_queue, OutputQueue* p_output_queue);
    virtual ~Tracker() = default;

    void fill_frame_queue(Frame::Ptr p_frame)
    {
        m_frame_queue.push(std::make_unique<Frame>(*p_frame));
    }

    virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

    virtual bool has_work() const override { return true; }


private:

    InputUniquePtr get_input_packet() override;
    //! Input Queues which are to be synchronised
    ThreadsafeQueue<Frame::UniquePtr> m_frame_queue;
    MaskedImageQueue* mp_masked_image_queue;

};
}// namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
