/******************************************************************************
 * File:             tracker.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker thread
 *****************************************************************************/
#include "tracker.h"

#include <Cuda/Common/UtilsCuda.h>
#include <Cuda/Geometry/GeometryClasses.h>
#include <Cuda/OSLAMUtils/SegmentationCuda.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>

#include <cstdlib>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>

#include "utils/utils.h"

namespace oslam
{
    Tracker::Tracker(RendererOutputQueue* renderer_output_queue, OutputQueue* output_queue)
        : MISO(output_queue, "Tracker"), frame_queue_("InputFrameQueue"), renderer_output_queue_(renderer_output_queue)
    {
        spdlog::debug("CONSTRUCT: Tracker");
    }

    Tracker::InputUniquePtr Tracker::getInputPacket()
    {
        auto start_time = Timer::tic();

        Frame::UniquePtr input_frame;
        bool queue_state = frame_queue_.popBlocking(input_frame);
        if (!input_frame || !queue_state)
        {
            spdlog::error("Module: {} {} returned null", name_id_, frame_queue_.queue_id_);
            return nullptr;
        }
        curr_timestamp_ = input_frame->timestamp_;

        Tracker::InputUniquePtr tracker_input;
        if (curr_timestamp_ == 1)
        {
            spdlog::debug("Tracker input payload: Timestamp: {}, Null Vertices, Null Normals", curr_timestamp_);
            tracker_input = std::make_unique<TrackerInput>(curr_timestamp_, *input_frame, cv::Mat(), cv::Mat(), cv::Mat());
        }
        else
        {
            RendererOutput::UniquePtr renderer_output;
            //! Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<RendererOutput::UniquePtr>(curr_timestamp_, renderer_output_queue_, &renderer_output))
            {
                spdlog::error("Missing Render output with requested timestamp: {}", curr_timestamp_);
                return nullptr;
            }
            if (!renderer_output)
            {
                spdlog::error("Module: {} {} returned null", name_id_, renderer_output_queue_->queue_id_);
                return nullptr;
            }
            tracker_input = std::make_unique<TrackerInput>(curr_timestamp_,
                                                           *input_frame,
                                                           renderer_output->colors_,
                                                           renderer_output->vertices_,
                                                           renderer_output->normals_);
        }
        if (!tracker_input)
        {
            spdlog::error("Unable to create TrackerInputPayload");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::debug("Processed tracker payload: {}, took {} ms", curr_timestamp_, duration);
        return tracker_input;
    }

    Tracker::OutputUniquePtr Tracker::runOnce(Tracker::InputUniquePtr input)
    {
        using namespace open3d;
        auto frame           = input->frame_;
        auto model_color_map = input->model_color_map_;
        auto model_vertices  = input->model_vertices_;
        auto model_normals   = input->model_normals_;

        frame_raw_depth_cuda_.Upload(frame.depth_);
        frame_color_cuda_.Upload(frame.color_);

        cuda::RGBDImageCuda frame_rgbd;
        frame_rgbd.Build(frame_raw_depth_cuda_, frame_color_cuda_);

        TrackerStatus tracker_status         = TrackerStatus::INVALID;
        Eigen::Matrix4d relative_camera_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix6d information_matrix   = Eigen::Matrix6d::Identity();
        Eigen::Matrix4d camera_pose;

        if (curr_timestamp_ == 1)
        {
            camera_pose    = relative_camera_pose;
            tracker_status = TrackerStatus::VALID;
        }
        else
        {
            auto odo_start_time = Timer::tic();

            cuda::ImageCuda<uchar, 3> model_color_map_cuda;
            cuda::ImageCuda<float, 3> model_vertices_cuda;
            cuda::ImageCuda<float, 3> model_normals_cuda;

            model_color_map_cuda.Upload(model_color_map);
            model_vertices_cuda.Upload(model_vertices);
            model_normals_cuda.Upload(model_normals);

            cuda::RGBDOdometryCuda<3> odometry;
            odometry.SetIntrinsics(frame.intrinsic_);
            odometry.SetParameters(odometry::OdometryOption({ 20, 10, 5 }, 0.07), 0.5f, cuda::OdometryType::FRAME_TO_MODEL);

            spdlog::debug("Initializing Odometry for frame: {}", curr_timestamp_);
            odometry.Initialize(frame_rgbd, model_vertices_cuda, model_normals_cuda, model_color_map_cuda);

            auto result = odometry.ComputeMultiScale();

            auto success         = std::get<0>(result);
            relative_camera_pose = std::get<1>(result);

            if (success)
                tracker_status = TrackerStatus::VALID;

            information_matrix   = odometry.ComputeInformationMatrix();
            auto odo_finish_time = Timer::toc(odo_start_time).count();
            spdlog::debug("Odometry took {} ms", odo_finish_time);
            camera_pose = prev_camera_pose * relative_camera_pose;
        }

        spdlog::debug("Current camera pose: \n{}\n", camera_pose);
        prev_camera_pose = camera_pose;

        return std::make_unique<TrackerOutput>(
            curr_timestamp_, tracker_status, frame, relative_camera_pose, information_matrix);
    }

    void Tracker::shutdownQueues()
    {
        MISOPipelineModule::shutdownQueues();
        frame_queue_.shutdown();
    }
}  // namespace oslam
