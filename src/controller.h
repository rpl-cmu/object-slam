/******************************************************************************
 * File:             controller.h
 *
 * Author:           Akash Sharma
 * Created:          04/13/20
 * Description:      Main loop for object-slam
 *****************************************************************************/
#ifndef OSLAM_CONTROLLER_H
#define OSLAM_CONTROLLER_H

#include <Open3D/Open3D.h>
#include <docopt/docopt.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <zmq.hpp>

#include "data_provider.h"
#include "data_reader.h"
#include "image_transport.h"
#include "map.h"
#include "tracker.h"
#include "utils/macros.h"
#include "utils/thread_class.h"
/* #include "visualizer.h" */

namespace oslam
{
  class Controller
  {
   public:
    OSLAM_DELETE_COPY_CONSTRUCTORS(Controller);
    explicit Controller(const std::map<std::string, docopt::value> &r_args);
    virtual ~Controller() = default;

    int start();

   private:
    bool setup();
    void run();
    bool shutdown_when_complete();
    void shutdown();

    bool m_visualize;
    bool m_debug;

    std::atomic_bool m_shutdown = { false };
    std::string m_dataset_path;

    std::shared_ptr<GlobalMap> mp_global_map;

    DataReader::Ptr mp_data_reader;
    DataProvider::Ptr mp_data_provider;
    ImageTransporter::Ptr mp_image_transport;
    Tracker::Ptr mp_tracker;

    ImageTransporter::InputQueue m_transport_input_queue;
    ImageTransporter::OutputQueue m_transport_output_queue;

    Tracker::OutputQueue m_tracker_output_queue;
  };
}  // namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
