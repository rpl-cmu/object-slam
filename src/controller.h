/******************************************************************************
 * File:             controller.h
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#ifndef OSLAM_CONTROLLER_H
#define OSLAM_CONTROLLER_H

#include <docopt/docopt.h>
#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>
#include <zmq.hpp>

#include "thread_class.h"
#include "dataset.h"
#include "image_transport.h"
#include "tracker.h"
/* #include "visualizer.h" */

namespace oslam {

class Controller
{
  public:
    Controller(const std::map<std::string, docopt::value>& r_arguments);
    virtual ~Controller() = default;

    int start(void);

  private:
    bool setup(void);
    void run(void);
    /* data */
    bool m_visualize;
    bool m_debug;
    std::string m_dataset_path;

    std::shared_ptr<RGBDdataset> mp_rgbd_dataset;
    std::shared_ptr<ImageTransporter> mp_image_transport;
    std::shared_ptr<Tracker> mp_tracker;

    std::vector<std::thread> mvp_threads;

    std::shared_ptr<zmq::context_t> mp_context;
};
}// namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
