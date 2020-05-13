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

    void fill_data_queue(std::unique_ptr<RGBDdata> p_data)
    {
        m_rgbd_data_queue.pushBlockingIfFull(std::move(p_data), 20u);
    }

  private:
    bool process(void) override;
    /* data */
    /* std::shared_ptr<oslam::RGBDdataset> mp_dataset; */
    std::shared_ptr<oslam::ImageTransporter> mp_image_transport;

    unsigned int m_curr_frame_id;

    ThreadsafeQueue<std::unique_ptr<RGBDdata>> m_rgbd_data_queue;

    std::shared_ptr<oslam::Frame> mp_current_frame;
    std::shared_ptr<oslam::Frame> mp_prev_frame;
};
}// namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
