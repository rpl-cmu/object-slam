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
#include <spdlog/spdlog.h>

#include <memory>
#include <zmq.hpp>

#include "data_provider.h"
#include "data_reader.h"
#include "image_transport.h"
#include "map.h"
#include "mapper.h"
#include "renderer.h"
#include "tracker.h"
#include "utils/macros.h"

namespace oslam
{
    /*! \class Controller
     *  \brief Creates and manages all the submodules in the SLAM System
     */
    class Controller
    {
       public:
        OSLAM_DELETE_COPY_CONSTRUCTORS(Controller);

        explicit Controller(const std::string& dataset_path);
        virtual ~Controller() = default;

        bool start();

       private:
        bool setup();
        void run();
        bool shutdownWhenComplete();
        void shutdown();

        std::string dataset_path_;
        std::atomic_bool shutdown_ = { false };

        std::shared_ptr<Map> map_;

        DataReader::Ptr data_reader_;
        DataProvider::Ptr data_provider_;
        ImageTransporter::Ptr image_transport_;
        Tracker::Ptr tracker_;
        Mapper::Ptr mapper_;
        Renderer::Ptr renderer_;

        ImageTransporter::InputQueue transport_input_queue_;
        ImageTransporter::OutputQueue transport_output_queue_;
        Tracker::OutputQueue tracker_output_queue_;
        Renderer::InputQueue renderer_input_queue_;
        Renderer::OutputQueue renderer_output_queue_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
