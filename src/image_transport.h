/******************************************************************************
 * File:             image_transport.h
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:      Transport image to and fro between cpp and python
 *****************************************************************************/
#ifndef OSLAM_IMAGE_TRANSPORT_H
#define OSLAM_IMAGE_TRANSPORT_H

#include <msg/image.pb.h>

#include <memory>
#include <thread>
#include <vector>
#include <zmq.hpp>

#include "frame.h"
#include "instance_image.h"
#include "utils/macros.h"
#include "utils/pipeline_module.h"
#include "utils/thread_safe_queue.h"

static constexpr auto SERVER_ENDPOINT = "tcp://localhost:5555";

namespace oslam
{
    /*! \class ImageTransporter
     *  \brief Transport frame object to Python process running PointRend inference for segmentation
     */
    class ImageTransporter : public SISOPipelineModule<Frame, ImageTransportOutput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(ImageTransporter);
        OSLAM_DELETE_COPY_CONSTRUCTORS(ImageTransporter);
        using SISO = SISOPipelineModule<Frame, ImageTransportOutput>;

        explicit ImageTransporter(SISO::InputQueue* input_queue, SISO::OutputQueue* output_queue);
        virtual ~ImageTransporter() = default;

        virtual ImageTransportOutput::UniquePtr runOnce(Frame::UniquePtr frame) override;

       protected:
        ImageTransportOutput::UniquePtr process(Frame::UniquePtr frame);

        //! \brief Serialize the image to send to server
        static std::string serializeFrame(const Frame& frame);

        //! \brief Deserialize the received image from server
        static ImageTransportOutput::UniquePtr deserializeFrame(Timestamp timestamp, const MaskImage& mask_pbuf);

        zmq::context_t context_{ 1 };  //!< Shared ZMQ context for thread to receive and send data
        zmq::socket_t req_sock_;       //!< ZMQ socket to send request to Python server
    };
}  // namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
