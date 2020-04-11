/******************************************************************************
 * File:             image_transport.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:
 *****************************************************************************/
#include "image_transport.h"
#include <msg/image.pb.h>
#include <zmq.hpp>

oslam::ImageTransporter::ImageTransporter(const ImageProperties &r_image_prop)
  : m_properties(r_image_prop), m_publisher(m_context, zmq::socket_type::pub)
{
    m_publisher.bind(PUBLISH_ENDPOINT);
    /* m_subscriber.bind(SUBSCRIBE_ENDPOINT); */
}

oslam::ImageTransporter::~ImageTransporter() {}

void oslam::ImageTransporter::send(const open3d::geometry::Image& r_color_image)
{

    oslam::ColorImage image_pbuf;
    image_pbuf.set_width(r_color_image.height_); // 480
    image_pbuf.set_height(r_color_image.width_); // 640
    image_pbuf.set_num_channels(r_color_image.num_of_channels_);
    image_pbuf.set_bytes_per_channel(r_color_image.bytes_per_channel_);
    image_pbuf.set_data(std::string(r_color_image.data_.begin(), r_color_image.data_.end()));

    std::string str;
    image_pbuf.SerializeToString(&str);
    zmq::message_t message(str.c_str(), str.length());
    m_publisher.send(message, zmq::send_flags::dontwait);
}
