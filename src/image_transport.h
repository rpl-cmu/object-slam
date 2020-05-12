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

#include "thread_class.h"

static constexpr auto PUBLISH_ENDPOINT = "tcp://*:4242";
static constexpr auto SUBSCRIBE_ENDPOINT = "tcp://localhost:4243";

namespace oslam {

struct ImageProperties
{
    int width;
    int height;
    int num_of_channels;
    int bytes_per_channel;
};

struct MaskedImage
{
    open3d::geometry::Image image;
    std::vector<unsigned int> labels;
    std::vector<double> scores;
};

class ImageTransporter : public Thread
{
  public:
    explicit ImageTransporter(const ImageProperties &r_image_prop, std::shared_ptr<zmq::context_t> p_context);
    ImageTransporter(const ImageTransporter&) = delete;
    ImageTransporter& operator=(const ImageTransporter&) = delete;

    virtual ~ImageTransporter();

    void send(const open3d::geometry::Image &r_color_image);

    std::unique_ptr<MaskedImage> p_masked_image;

  private:
    bool process(void) override;
    void image_callback(const MaskImage &r_mask_pbuf);

    ImageProperties m_properties;

    std::shared_ptr<zmq::context_t> mp_context;
    zmq::socket_t m_publisher;
    zmq::socket_t m_subscriber;

};
}// namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
