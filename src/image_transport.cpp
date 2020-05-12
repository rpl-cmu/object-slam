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

ImageTransporter::ImageTransporter(const ImageProperties &r_image_prop, std::shared_ptr<zmq::context_t> p_context)
  : Thread("ImageTransportThread"), m_properties(r_image_prop), mp_context(p_context), m_publisher(*mp_context, zmq::socket_type::pub),
    m_subscriber(*mp_context, zmq::socket_type::sub)
{
    m_publisher.bind(PUBLISH_ENDPOINT);
    m_subscriber.connect(SUBSCRIBE_ENDPOINT);
    m_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

ImageTransporter::~ImageTransporter() {}

void ImageTransporter::send(const open3d::geometry::Image &r_color_image)
{
    oslam::ColorImage image_pbuf;
    image_pbuf.set_width(r_color_image.height_);// 480
    image_pbuf.set_height(r_color_image.width_);// 640
    image_pbuf.set_num_channels(r_color_image.num_of_channels_);
    image_pbuf.set_bytes_per_channel(r_color_image.bytes_per_channel_);
    image_pbuf.set_data(std::string(r_color_image.data_.begin(), r_color_image.data_.end()));

    std::string str;
    image_pbuf.SerializeToString(&str);
    zmq::message_t message(str.c_str(), str.length());
    m_publisher.send(message, zmq::send_flags::dontwait);
}

void ImageTransporter::image_callback(const oslam::MaskImage &mask_pbuf)
{
    std::scoped_lock<std::mutex> lock(m_mutex);

    p_masked_image = std::unique_ptr<MaskedImage>(new MaskedImage);
    p_masked_image->image.width_ = mask_pbuf.width();
    p_masked_image->image.height_ = mask_pbuf.height();
    p_masked_image->image.num_of_channels_ = mask_pbuf.num_channels();
    p_masked_image->image.bytes_per_channel_ = mask_pbuf.bytes_per_channel();
    const std::string image_data(mask_pbuf.data());
    p_masked_image->image.data_.assign(image_data.begin(), image_data.end());

    p_masked_image->labels.assign(mask_pbuf.labels().begin(), mask_pbuf.labels().end());
    p_masked_image->scores.assign(mask_pbuf.scores().begin(), mask_pbuf.scores().end());
    spdlog::info("Callback received");
}

bool ImageTransporter::process()
{
    zmq::message_t recv_msg;
    oslam::MaskImage mask_pbuf;
    auto success = m_subscriber.recv(recv_msg, zmq::recv_flags::dontwait);
    if (success) {
        mask_pbuf.ParseFromString(recv_msg.to_string());
        image_callback(mask_pbuf);
    }
    spdlog::info("Callback sent");
    return true;
}

}// namespace oslam
