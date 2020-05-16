/******************************************************************************
 * File:             image_transport.h
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:      Transport image to and fro between cpp and python
 *****************************************************************************/
#ifndef OSLAM_IMAGE_TRANSPORT_H
#define OSLAM_IMAGE_TRANSPORT_H

#include <Open3D/Open3D.h>
#include <memory>
#include <thread>
#include <vector>
#include <zmq.hpp>

#include <msg/image.pb.h>

#include "utils/macros.h"
#include "utils/pipeline_module.h"
#include "utils/thread_class.h"
#include "utils/thread_safe_queue.h"

#include "frame.h"

static constexpr auto SERVER_ENDPOINT = "tcp://localhost:5555";

namespace oslam {

/*! \class ImageTransporter
 *  \brief Transport frame object to Python process running PointRend inference for segmentation
 *
 *  Detailed description
 */
class ImageTransporter : public SISOPipelineModule<Frame, MaskedImage>
{
  public:
    OSLAM_POINTER_TYPEDEFS(ImageTransporter);
    OSLAM_DELETE_COPY_CONSTRUCTORS(ImageTransporter);

    using SISO = SISOPipelineModule<Frame, MaskedImage>;

    explicit ImageTransporter(SISO::InputQueue* p_input_queue, SISO::OutputQueue* p_output_queue);
    virtual ~ImageTransporter() = default;

    MaskedImage::UniquePtr run_once(Frame::UniquePtr p_frame);
  protected:

    MaskedImage::UniquePtr process(Frame::UniquePtr p_frame);

    //! Serialize the image to send to server
    std::string serialize_image(const open3d::geometry::Image &r_color_image);

    //! Deserialize the received image from server
    MaskedImage::UniquePtr deserialize_image(Timestamp timestamp, const MaskImage &r_mask_pbuf);


    //! Shared ZMQ context for this thread to receive and send data
    zmq::context_t m_context{ 1 };
    //! ZMQ socket to send request to Python server
    zmq::socket_t m_request_sock;
};
}// namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
