/******************************************************************************
 * File:             tracker.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker thread
 *****************************************************************************/
#include "tracker.h"

#include <cstdlib>
#include <memory>

#include <Cuda/Common/UtilsCuda.h>
#include <Cuda/Geometry/GeometryClasses.h>
#include <Cuda/OSLAMUtils/SegmentationCuda.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>

#include "object-slam/utils/utils.h"

namespace oslam
{
    Tracker::Tracker(OutputQueue* output_queue)
        : MISO(output_queue, "Tracker"), frame_queue_("TrackerFrameQueue"), model_queue_("TrackerModelQueue")
    {
        spdlog::trace("CONSTRUCT: Tracker");
    }

    Tracker::InputUniquePtr Tracker::getInputPacket()
    {
        auto start_time = Timer::tic();

        Frame::UniquePtr input_frame = nullptr;
        bool queue_state             = frame_queue_.popBlocking(input_frame);
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
            Model::UniquePtr model;
            //! Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<Model::UniquePtr>(curr_timestamp_, &model_queue_, &model))
            {
                spdlog::error("Missing Render output with requested timestamp: {}", curr_timestamp_);
                return nullptr;
            }
            if (!model)
            {
                spdlog::error("Module: {} {} returned null", name_id_, model_queue_.queue_id_);
                return nullptr;
            }
            tracker_input = std::make_unique<TrackerInput>(
                curr_timestamp_, *input_frame, model->colors_, model->vertices_, model->normals_);
        }
        if (!tracker_input)
        {
            spdlog::error("Unable to create TrackerInputPayload");
            return nullptr;
        }
        spdlog::info("Processed tracker payload: {}, took {} ms", curr_timestamp_, Timer::toc(start_time).count());
        return tracker_input;
    }

    Tracker::OutputUniquePtr Tracker::runOnce(Tracker::InputUniquePtr input)
    {
        using namespace open3d;
        const auto& frame           = input->frame_;
        const auto& model_color_map = input->model_color_map_;
        const auto& model_vertices  = input->model_vertices_;
        const auto& model_normals   = input->model_normals_;

        cuda::ImageCuda<ushort, 1> frame_raw_depth_cuda;
        cuda::ImageCuda<uchar, 3> frame_color_cuda;

        frame_raw_depth_cuda.Upload(frame.depth_);
        frame_color_cuda.Upload(frame.color_);

        cuda::RGBDImageCuda frame_rgbd(frame.max_depth_, frame.depth_factor_);
        frame_rgbd.Build(frame_raw_depth_cuda, frame_color_cuda);

        TrackerStatus tracker_status         = TrackerStatus::INVALID;
        Eigen::Matrix4d relative_camera_pose = Eigen::Matrix4d::Identity();
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
            odometry.SetParameters(odometry::OdometryOption({ 20, 10, 5 }, 0.07), 0.5F, cuda::OdometryType::FRAME_TO_MODEL);

            spdlog::debug("Initializing Odometry for frame: {}", curr_timestamp_);
            odometry.Initialize(frame_rgbd, model_vertices_cuda, model_normals_cuda, model_color_map_cuda);

            auto result = odometry.ComputeMultiScale();

            auto success         = std::get<0>(result);
            relative_camera_pose = std::get<1>(result);

            if (success)
            {
                tracker_status = TrackerStatus::VALID;
            }

            spdlog::info("Odometry took {} ms", Timer::toc(odo_start_time).count());
            camera_pose = prev_camera_pose * relative_camera_pose;
        }

        spdlog::info("Current camera pose: \n{}\n", camera_pose);
        prev_camera_pose = camera_pose;

        return std::make_unique<TrackerOutput>(curr_timestamp_, tracker_status, frame, relative_camera_pose);
    }

    void Tracker::shutdownQueues()
    {
        MISOPipelineModule::shutdownQueues();
        frame_queue_.shutdown();
        model_queue_.shutdown();
    }
}  // namespace oslam
