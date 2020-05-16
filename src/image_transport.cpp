/******************************************************************************
 * File:             image_transport.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:      ImageTransporter Module implementation
 *****************************************************************************/
#include "image_transport.h"

#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <zmq.hpp>

namespace oslam {

ImageTransporter::ImageTransporter(SISO::InputQueue* p_input_queue, SISO::OutputQueue* p_output_queue)
    :SISO(p_input_queue, p_output_queue, "ImageTransporterModule"), m_request_sock(m_context, ZMQ_REQ)
{
    m_request_sock.connect(SERVER_ENDPOINT);
    spdlog::info("Connected C++ client to ENDPOINT: {}", SERVER_ENDPOINT);
}

ImageTransporter::OutputUniquePtr ImageTransporter::run_once(ImageTransporter::InputUniquePtr p_frame)
{
    if(p_frame)
    {
        MaskedImage::UniquePtr p_masked_image = process(std::move(p_frame));
        if(p_masked_image)
            return p_masked_image;
    }
    return nullptr;
}

MaskedImage::UniquePtr ImageTransporter::process(Frame::UniquePtr p_frame)
{
    std::string str = serialize_image(p_frame->get_color());
    Timestamp curr_timestamp = p_frame->m_timestamp;

    zmq::message_t message(str.c_str(), str.length());
    m_request_sock.send(message, zmq::send_flags::dontwait);

    spdlog::debug("Send request from client");
    zmq::message_t recv_msg;
    oslam::MaskImage mask_pbuf;

    //Blocking call
    auto success = m_request_sock.recv(recv_msg);
    if (success) {
        mask_pbuf.ParseFromString(recv_msg.to_string());
        std::unique_ptr<MaskedImage> p_masked_image = deserialize_image(curr_timestamp, mask_pbuf);
        //TODO: Send output to all callbacks
        if(p_masked_image)
            return p_masked_image;
    }
    return nullptr;
}

std::string ImageTransporter::serialize_image(const open3d::geometry::Image& r_color_image)
{
    oslam::ColorImage image_pbuf;
    image_pbuf.set_width(r_color_image.height_);// 480
    image_pbuf.set_height(r_color_image.width_);// 640
    image_pbuf.set_num_channels(r_color_image.num_of_channels_);
    image_pbuf.set_bytes_per_channel(r_color_image.bytes_per_channel_);
    image_pbuf.set_data(std::string(r_color_image.data_.begin(), r_color_image.data_.end()));

    std::string str;
    image_pbuf.SerializeToString(&str);
    return str;
}

MaskedImage::UniquePtr ImageTransporter::deserialize_image(const Timestamp timestamp, const oslam::MaskImage &mask_pbuf)
{
    const std::string image_data(mask_pbuf.data());

    open3d::geometry::Image image;
    image.width_ = mask_pbuf.width();
    image.height_ = mask_pbuf.height();
    image.num_of_channels_ = mask_pbuf.num_channels();
    image.bytes_per_channel_ = mask_pbuf.bytes_per_channel();
    image.data_.assign(image_data.begin(), image_data.end());

    std::vector<unsigned int> labels; labels.assign(mask_pbuf.labels().begin(), mask_pbuf.labels().end());
    std::vector<double> scores; scores.assign(mask_pbuf.scores().begin(), mask_pbuf.scores().end());

    MaskedImage::UniquePtr p_masked_image = std::make_unique<MaskedImage>(timestamp, image, labels, scores);
    return p_masked_image;
}

}// namespace oslam
