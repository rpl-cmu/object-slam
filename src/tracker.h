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

#include "thread_class.h"
#include "dataset.h"
#include "image_transport.h"
#include "frame.h"


namespace oslam {
class Tracker: public Thread
{
  public:
    explicit Tracker(std::shared_ptr<oslam::RGBDdataset> p_rgbd_dataset,
      std::shared_ptr<oslam::ImageTransporter> p_image_transport);
    virtual ~Tracker();

  private:
    bool process(void) override;
    /* data */
    std::shared_ptr<oslam::RGBDdataset> mp_dataset;
    std::shared_ptr<oslam::ImageTransporter> mp_image_transport;

    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    unsigned int m_curr_frame_id;

    std::shared_ptr<oslam::Frame> mp_current_frame;
    std::shared_ptr<oslam::Frame> mp_prev_frame;
};
}// namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
