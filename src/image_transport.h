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
     *
     *  Detailed description
     */
    class ImageTransporter : public SISOPipelineModule<Frame, ImageTransportOutput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(ImageTransporter);
        OSLAM_DELETE_COPY_CONSTRUCTORS(ImageTransporter);

        using SISO = SISOPipelineModule<Frame, ImageTransportOutput>;

        explicit ImageTransporter(SISO::InputQueue* p_input_queue, SISO::OutputQueue* p_output_queue);
        virtual ~ImageTransporter() = default;

        virtual OutputUniquePtr run_once(InputUniquePtr p_frame) override;

       protected:
        ImageTransportOutput::UniquePtr process(Frame::UniquePtr p_frame);

        //! Serialize the image to send to server
        std::string serialize_frame(const Frame& r_frame);

        //! Deserialize the received image from server
        ImageTransportOutput::UniquePtr deserialize_frame(Timestamp timestamp, const MaskImage& r_mask_pbuf);

        //! Shared ZMQ context for this thread to receive and send data
        zmq::context_t m_context{ 1 };
        //! ZMQ socket to send request to Python server
        zmq::socket_t m_request_sock;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
