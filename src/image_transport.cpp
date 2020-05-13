/******************************************************************************
 * File:             image_transport.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:
 *****************************************************************************/
#include "image_transport.h"

#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <zmq.hpp>

namespace oslam {

ImageTransporter::ImageTransporter()
    :m_request_sock(m_context, ZMQ_REQ), m_send_frame_queue("SendFrameQueue")
{
    m_request_sock.connect(SERVER_ENDPOINT);
    spdlog::info("Connected C++ client to ENDPOINT: {}", SERVER_ENDPOINT);
}

Frame::UniquePtr ImageTransporter::getInput(void)
{
    Frame::UniquePtr input_frame = nullptr;
    bool queue_state = false;
    queue_state = m_send_frame_queue.popBlocking(input_frame);
    if (queue_state)
        return input_frame;
    else
        spdlog::error("Input Frame Queue is down");

    return nullptr;

}

bool ImageTransporter::run()
{
    spdlog::info("Thread ({},{}) started", "ImageTransporterThread", std::this_thread::get_id());
    while(!m_shutdown)
    {
        Frame::UniquePtr p_frame = getInput();

        if(p_frame)
        {
            MaskedImage::UniquePtr p_masked_image = process(std::move(p_frame));

            for(const MaskedImageCallback& r_callback: m_output_callbacks)
                r_callback(std::move(p_masked_image));
        }
    }
    spdlog::info("Thread ({}, {}), ended", "ImageTransporterThread", std::this_thread::get_id());
    return false;
}

MaskedImage::UniquePtr ImageTransporter::process(Frame::UniquePtr p_frame)
{
    open3d::geometry::Image color_image = p_frame->get_color_image();

    // Serialise the color image to send
    oslam::ColorImage image_pbuf;
    image_pbuf.set_width(color_image.height_);// 480
    image_pbuf.set_height(color_image.width_);// 640
    image_pbuf.set_num_channels(color_image.num_of_channels_);
    image_pbuf.set_bytes_per_channel(color_image.bytes_per_channel_);
    image_pbuf.set_data(std::string(color_image.data_.begin(), color_image.data_.end()));

    std::string str;
    image_pbuf.SerializeToString(&str);
    zmq::message_t message(str.c_str(), str.length());
    m_request_sock.send(message, zmq::send_flags::dontwait);

    spdlog::debug("Send request from client");
    zmq::message_t recv_msg;
    oslam::MaskImage mask_pbuf;

    //Blocking call
    auto success = m_request_sock.recv(recv_msg);
    if (success) {
        mask_pbuf.ParseFromString(recv_msg.to_string());
        std::unique_ptr<MaskedImage> p_masked_image = deserialize_image(mask_pbuf);
        //TODO: Send output to all callbacks
        if(p_masked_image)
            return p_masked_image;
    }
    return nullptr;


}


/* void ImageTransporter::send(const open3d::geometry::Image &r_color_image) */
/* { */
/*     oslam::ColorImage image_pbuf; */
/*     image_pbuf.set_width(r_color_image.height_);// 480 */
/*     image_pbuf.set_height(r_color_image.width_);// 640 */
/*     image_pbuf.set_num_channels(r_color_image.num_of_channels_); */
/*     image_pbuf.set_bytes_per_channel(r_color_image.bytes_per_channel_); */
/*     image_pbuf.set_data(std::string(r_color_image.data_.begin(), r_color_image.data_.end())); */

/*     std::string str; */
/*     image_pbuf.SerializeToString(&str); */
/*     zmq::message_t message(str.c_str(), str.length()); */
/*     m_publisher.send(message, zmq::send_flags::dontwait); */
/* } */
MaskedImage::UniquePtr ImageTransporter::deserialize_image(const oslam::MaskImage &mask_pbuf)
{
    MaskedImage::UniquePtr p_masked_image = std::make_unique<MaskedImage>();
    p_masked_image->image.width_ = mask_pbuf.width();
    p_masked_image->image.height_ = mask_pbuf.height();
    p_masked_image->image.num_of_channels_ = mask_pbuf.num_channels();
    p_masked_image->image.bytes_per_channel_ = mask_pbuf.bytes_per_channel();
    const std::string image_data(mask_pbuf.data());
    p_masked_image->image.data_.assign(image_data.begin(), image_data.end());

    p_masked_image->labels.assign(mask_pbuf.labels().begin(), mask_pbuf.labels().end());
    p_masked_image->scores.assign(mask_pbuf.scores().begin(), mask_pbuf.scores().end());
    return p_masked_image;
}

}// namespace oslam
