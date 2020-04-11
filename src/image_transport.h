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
#include <zmq.hpp>

static constexpr auto PUBLISH_ENDPOINT = "tcp://*:4242";
static constexpr auto SUBSCRIBE_ENDPOINT = "tcp://*.4243";

namespace oslam {

struct ImageProperties {
    int width;
    int height;
    int num_of_channels;
    int bytes_per_channel;
};
class ImageTransporter
{
public:
    explicit ImageTransporter (const ImageProperties& r_image_prop);
    virtual ~ImageTransporter ();

    void send(const open3d::geometry::Image& r_color_image);

private:
    /* data */
    ImageProperties m_properties;
    zmq::context_t m_context;
    zmq::socket_t m_publisher;
    /* zmq::socket_t m_subscriber; */
};
} // namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
