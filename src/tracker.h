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

#include "utils/thread_class.h"
#include "data_reader.h"
#include "image_transport.h"
#include "frame.h"
#include "utils/thread_safe_queue.h"


namespace oslam {
class Tracker: public Thread
{
  public:
    explicit Tracker(std::shared_ptr<oslam::ImageTransporter> p_image_transport);
    virtual ~Tracker();

    //! Callbacks
    void fill_data_queue(Frame::UniquePtr p_frame)
    {
        m_input_frame_queue.pushBlockingIfFull(std::move(p_frame), 20u);
    }

    void fill_mask_image_queue(MaskedImage::UniquePtr p_masked_image)
    {
        m_input_mask_queue.pushBlockingIfFull(std::move(p_masked_image), 20u);
    }

    //TODO: move the FrameCallback typedef to frame class
    void register_callback(const DataReader::FrameCallback& r_callback)
    {
        m_transport_callback = r_callback;
    }

  private:
    Frame::UniquePtr getInput(void);
    bool process(void) override;

    std::shared_ptr<oslam::ImageTransporter> mp_image_transport;

    unsigned int m_curr_frame_id;

    ThreadsafeQueue<Frame::UniquePtr> m_input_frame_queue;
    ThreadsafeQueue<MaskedImage::UniquePtr> m_input_mask_queue;

    //! Calls the ImageTransporter object to fill its queue
    DataReader::FrameCallback m_transport_callback;

    //TODO(Akash): Change this to UniquePtrs later for sanity
    Frame::UniquePtr mp_current_frame;
    Frame::UniquePtr mp_prev_frame;
};
}// namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
