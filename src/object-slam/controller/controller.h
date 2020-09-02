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

#include "object-slam/struct/map.h"
#include "object-slam/utils/macros.h"

#include "object-slam/module/image_transport.h"
#include "object-slam/module/tracker.h"
#include "object-slam/module/mapper.h"
#include "object-slam/module/renderer.h"
#include "object-slam/module/display.h"
#include "object-slam/reader/data_provider.h"
#include "object-slam/reader/data_reader.h"

namespace oslam
{
    /*! \class Controller
     *  \brief Creates and manages all the submodules in the SLAM System
     */
    class Controller
    {
       public:
        OSLAM_DELETE_COPY_CONSTRUCTORS(Controller);
        OSLAM_DELETE_MOVE_CONSTRUCTORS(Controller);

        explicit Controller(std::string dataset_path);
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
        Display::Ptr display_;

        ImageTransporter::InputQueue transport_input_queue_;
        ImageTransporter::OutputQueue transport_output_queue_;
        Tracker::OutputQueue tracker_output_queue_;
        Renderer::InputQueue renderer_input_queue_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_CONTROLLER_H */
