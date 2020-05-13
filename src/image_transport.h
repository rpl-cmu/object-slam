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
class ImageTransporter
{
public:
    //TODO(Akash): Change this to modified frame
    typedef std::function<void(MaskedImage::UniquePtr)> MaskedImageCallback;
    explicit ImageTransporter();
    virtual ~ImageTransporter() = default;

    //! Non-copyable and Non-assignable by default
    ImageTransporter(const ImageTransporter&) = delete;
    ImageTransporter& operator=(const ImageTransporter&) = delete;

    void register_callback(const MaskedImageCallback& r_callback)
    {
        m_output_callbacks.push_back(r_callback);
    }

    inline void fill_send_frame_queue(Frame::UniquePtr p_frame)
    {
        if(p_frame->is_keyframe())
            m_send_frame_queue.pushBlockingIfFull(std::move(p_frame));
    }

    bool run(void);

protected:

    Frame::UniquePtr getInput(void);

    MaskedImage::UniquePtr process(Frame::UniquePtr p_frame);

    MaskedImage::UniquePtr deserialize_image(const MaskImage &r_mask_pbuf);

    //! Shared ZMQ context for this thread to receive and send data
    zmq::context_t m_context{1};
    //! ZMQ socket to send request to Python server
    zmq::socket_t m_request_sock;

    //! Frame buffer to send
    ThreadsafeQueue<Frame::UniquePtr> m_send_frame_queue;

    //! Output callbacks
    std::vector<MaskedImageCallback> m_output_callbacks;

    //! Atomic boolean to control clean shutdown of thread
    std::atomic_bool m_shutdown = {false};
};
}// namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
