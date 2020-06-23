/******************************************************************************
 * File:             image_transport.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/11/20
 * Description:      ImageTransporter Module implementation
 *****************************************************************************/
#include "image_transport.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <zmq.hpp>

namespace oslam
{
    ImageTransporter::ImageTransporter(SISO::InputQueue* p_input_queue, SISO::OutputQueue* p_output_queue)
        : SISO(p_input_queue, p_output_queue, "ImageTransporter"), m_request_sock(m_context, ZMQ_REQ)
    {
        m_request_sock.connect(SERVER_ENDPOINT);
        spdlog::info("Connected C++ client to ENDPOINT: {}", SERVER_ENDPOINT);
    }

    ImageTransporter::OutputUniquePtr ImageTransporter::run_once(ImageTransporter::InputUniquePtr p_frame)
    {
        if (p_frame)
        {
            return process(std::move(p_frame));
        }
        return nullptr;
    }

    ImageTransportOutput::UniquePtr ImageTransporter::process(Frame::UniquePtr p_frame)
    {
        std::string str          = serialize_frame(*p_frame);
        Timestamp curr_timestamp = p_frame->m_timestamp;

        zmq::message_t message(str.c_str(), str.length());
        m_request_sock.send(message, zmq::send_flags::dontwait);

        spdlog::debug("Send request from client");
        zmq::message_t recv_msg;
        oslam::MaskImage mask_pbuf;

        // TODO(Akash): Blocking call!! Can we do this asynchronously?
        auto success = m_request_sock.recv(recv_msg);
        if (success)
        {
            mask_pbuf.ParseFromString(recv_msg.to_string());
            ImageTransportOutput::UniquePtr p_output = deserialize_frame(curr_timestamp, mask_pbuf);
            if (p_output)
                return p_output;
        }
        return nullptr;
    }

    std::string ImageTransporter::serialize_frame(const Frame& r_frame)
    {
        oslam::ColorImage image_pbuf;
        image_pbuf.set_width(r_frame.m_width);    // 640
        image_pbuf.set_height(r_frame.m_height);  // 480
        image_pbuf.set_num_channels(r_frame.m_color.channels());
        image_pbuf.set_bytes_per_channel(1);  // CV_8UC3
        image_pbuf.set_data(std::string(r_frame.m_color.datastart, r_frame.m_color.dataend));

        std::string str;
        image_pbuf.SerializeToString(&str);
        return str;
    }

    ImageTransportOutput::UniquePtr ImageTransporter::deserialize_frame(const Timestamp timestamp,
                                                                  const oslam::MaskImage& mask_pbuf)
    {
        std::string input_data(mask_pbuf.data());
        cv::Mat image(mask_pbuf.height(), mask_pbuf.width(), CV_16UC1, input_data.data());

        ImageTransportOutput::UniquePtr p_output = std::make_unique<ImageTransportOutput>(timestamp);
        for (int i = 0; i < mask_pbuf.labels_size(); i++)
        {
            unsigned int curr_label = mask_pbuf.labels(i);
            double curr_score       = mask_pbuf.scores(i);
            BoundingBox curr_bbox   = { mask_pbuf.bboxes(i).coordinates(0), mask_pbuf.bboxes(i).coordinates(1),
                                      mask_pbuf.bboxes(i).coordinates(2), mask_pbuf.bboxes(i).coordinates(3) };
            cv::Mat curr_mask;
            cv::bitwise_and(image, static_cast<std::uint16_t>(1u << i), curr_mask);
            curr_mask = (curr_mask >= 1);
            p_output->m_instance_images.emplace_back(curr_mask, curr_bbox, curr_label, curr_score);
        }
        return p_output;
    }
}  // namespace oslam
