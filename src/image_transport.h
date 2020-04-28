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
class ImageTransporter
{
  public:
    explicit ImageTransporter(const ImageProperties &r_image_prop);
    virtual ~ImageTransporter();

    void poll(void);
    void send(const open3d::geometry::Image &r_color_image);
    void image_callback(const MaskImage &r_mask_pbuf);

    std::unique_ptr<MaskedImage> p_masked_image;

  private:
    ImageProperties m_properties;
    zmq::context_t m_context;
    zmq::socket_t m_publisher;
    zmq::socket_t m_subscriber;

    std::thread m_thread;
    std::mutex m_mutex;
};
}// namespace oslam
#endif /* ifndef OSLAM_IMAGE_TRANSPORT_H */
