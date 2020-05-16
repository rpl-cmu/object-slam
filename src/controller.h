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
#include <memory>
#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>
#include <zmq.hpp>

#include "utils/macros.h"
#include "utils/thread_class.h"
#include "data_reader.h"
#include "data_provider.h"
#include "image_transport.h"
#include "tracker.h"
#include "map.h"
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

    std::shared_ptr<GlobalMap> mp_global_map;

    DataReader::Ptr mp_data_reader;
    DataProvider::Ptr mp_data_provider;
    ImageTransporter::Ptr mp_image_transport;
    Tracker::Ptr mp_tracker;

    DataProvider::InputQueue m_data_provider_input_queue;
    ImageTransporter::InputQueue m_transport_frame_queue;
    ImageTransporter::OutputQueue m_masked_image_queue;


};
}// namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
