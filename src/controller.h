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

#include "dataset.h"
#include "frame.h"
#include "image_transport.h"

namespace oslam {

class Controller
{
  public:
    Controller(const std::map<std::string, docopt::value>& r_arguments);
    virtual ~Controller() = default;

    void run(void);

  private:
    /* data */
    std::string m_dataset_path;
    bool m_visualize;
    bool m_debug;

    std::unique_ptr<RGBDdataset> mp_rgbd_dataset;
    std::unique_ptr<ImageTransporter> mp_image_transport;

};
}// namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
