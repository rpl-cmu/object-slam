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
#include <xtensor/xadapt.hpp>
#include <xtensor/xview.hpp>
#include <zmq.hpp>

namespace oslam
{
    ImageTransporter::ImageTransporter(SISO::InputQueue* input_queue, SISO::OutputQueue* output_queue)
        : SISO(input_queue, output_queue, "ImageTransporter"), req_sock_(context_, ZMQ_REQ)
    {
        spdlog::debug("CONSTRUCT: ImageTransporter");
        req_sock_.connect(SERVER_ENDPOINT);
        spdlog::info("Connected C++ client to ENDPOINT: {}", SERVER_ENDPOINT);
    }

    ImageTransportOutput::UniquePtr ImageTransporter::runOnce(Frame::UniquePtr frame)
    {
        if (frame)
        {
            return process(std::move(frame));
        }
        return nullptr;
    }

    ImageTransportOutput::UniquePtr ImageTransporter::process(Frame::UniquePtr frame)
    {
        std::string str          = serializeFrame(*frame);
        Timestamp curr_timestamp = frame->timestamp_;

        zmq::message_t message(str.c_str(), str.length());
        req_sock_.send(message, zmq::send_flags::dontwait);
        spdlog::debug("Send request from client");
        zmq::message_t recv_msg;
        oslam::MaskImage mask_pbuf;

        // Blocking call.
        auto success = req_sock_.recv(recv_msg);
        if (success)
        {
            mask_pbuf.ParseFromString(recv_msg.to_string());
            ImageTransportOutput::UniquePtr output = deserializeFrame(curr_timestamp, mask_pbuf);
            if (output)
                return output;
        }
        return nullptr;
    }

    std::string ImageTransporter::serializeFrame(const Frame& frame)
    {
        oslam::ColorImage image_pbuf;
        image_pbuf.set_width(frame.width_);    // 640
        image_pbuf.set_height(frame.height_);  // 480
        image_pbuf.set_num_channels(frame.color_.channels());
        image_pbuf.set_bytes_per_channel(1);  // CV_8UC3
        image_pbuf.set_data(std::string(frame.color_.datastart, frame.color_.dataend));

        std::string str;
        image_pbuf.SerializeToString(&str);
        return str;
    }

    ImageTransportOutput::UniquePtr ImageTransporter::deserializeFrame(const Timestamp timestamp,
                                                                       const oslam::MaskImage& mask_pbuf)
    {
        std::string input_data(mask_pbuf.data());
        cv::Mat image(mask_pbuf.height(), mask_pbuf.width(), CV_16UC1, input_data.data());

        std::string input_features(mask_pbuf.features());
        cv::Mat feature_vector(1024, mask_pbuf.labels_size(), CV_32FC1, input_features.data());
        ImageTransportOutput::UniquePtr output = std::make_unique<ImageTransportOutput>(timestamp);
        for (int i = 0; i < mask_pbuf.labels_size(); i++)
        {
            unsigned int curr_label = mask_pbuf.labels(i);
            double curr_score       = mask_pbuf.scores(i);
            BoundingBox curr_bbox   = { static_cast<int>(mask_pbuf.bboxes(i).coordinates(0)),
                                      static_cast<int>(mask_pbuf.bboxes(i).coordinates(1)),
                                      static_cast<int>(mask_pbuf.bboxes(i).coordinates(2)),
                                      static_cast<int>(mask_pbuf.bboxes(i).coordinates(3)) };
            cv::Mat curr_mask;
            cv::bitwise_and(image, static_cast<std::uint16_t>(1U << i), curr_mask);
            curr_mask = (curr_mask >= 1);

            Feature feature = feature_vector.col(i);
            output->instance_images_.emplace_back(curr_mask,  curr_bbox, curr_label, curr_score, feature);
        }
        return output;
    }
}  // namespace oslam
