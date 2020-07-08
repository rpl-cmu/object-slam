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
    Tracker::Tracker(RendererOutputQueue *p_renderer_output_queue, OutputQueue *p_output_queue)
        : MISO(p_output_queue, "Tracker"),
          m_frame_queue("InputFrameQueue"),
          mp_renderer_output_queue(p_renderer_output_queue),
          mr_global_map(GlobalMap::get_instance())
    {
    }

    Tracker::InputUniquePtr Tracker::get_input_packet()
    {
        auto start_time = Timer::tic();

        Frame::UniquePtr p_input_frame;
        bool queue_state = m_frame_queue.popBlocking(p_input_frame);
        if (!p_input_frame || !queue_state)
        {
            spdlog::error("Module: {} {} returned null", name_id_, m_frame_queue.queue_id_);
        }

        m_curr_timestamp = p_input_frame->m_timestamp;

        Tracker::InputUniquePtr p_tracker_input;
        if (m_curr_timestamp == 1)
        {
            spdlog::debug("Tracker input payload: Timestamp: {}, Null Vertices, Null Normals", m_curr_timestamp);
            //!
            p_tracker_input =
                std::make_unique<TrackerInput>(m_curr_timestamp, *p_input_frame, cv::Mat(), cv::Mat(), cv::Mat());
        }
        else
        {
            RendererOutput::UniquePtr p_renderer_output;
            //! Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<RendererOutput::UniquePtr>(m_curr_timestamp, mp_renderer_output_queue, &p_renderer_output))
            {
                spdlog::error("Missing Render output with requested timestamp: {}", m_curr_timestamp);
                return nullptr;
            }
            if (!p_renderer_output)
            {
                spdlog::error("Module: {} {} returned null", name_id_, mp_renderer_output_queue->queue_id_);
                return nullptr;
            }
            p_tracker_input = std::make_unique<TrackerInput>(
                m_curr_timestamp,
                *p_input_frame,
                p_renderer_output->m_colors,
                p_renderer_output->m_vertices,
                p_renderer_output->m_normals);
        }
        if (!p_tracker_input)
        {
            spdlog::error("Unable to create TrackerInputPayload");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::info("Processed tracker payload: {}, took {} ms", m_curr_timestamp, duration);

        return p_tracker_input;
    }

    Tracker::OutputUniquePtr Tracker::run_once(Tracker::InputUniquePtr p_input)
    {
        using namespace open3d;
        if (m_first_run)
        {
            CheckCuda(cudaSetDevice(0));
            m_first_run = false;
            mc_intrinsic.SetIntrinsics(p_input->m_frame.m_intrinsic);
        }

        auto curr_frame            = p_input->m_frame;
        auto curr_global_color_map = p_input->m_global_color_map;
        auto curr_global_vertices  = p_input->m_global_vertices;
        auto curr_global_normals   = p_input->m_global_normals;

        //! Upload current color and depth frame
        mc_curr_depth_raw.Upload(curr_frame.m_depth);
        mc_curr_color.Upload(curr_frame.m_color);

        //! Create vertex and normal maps from current frame
        cuda::RGBDImageCuda source_rgbd;
        source_rgbd.Build(mc_curr_depth_raw, mc_curr_color);

        TrackerStatus curr_tracker_status         = TrackerStatus::INVALID;
        Eigen::Matrix4d curr_relative_camera_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix6d curr_information_matrix   = Eigen::Matrix6d::Identity();

        Eigen::Matrix4d current_odometry;
        if (m_curr_timestamp == 1)
        {
            current_odometry = curr_relative_camera_pose;
            mv_T_camera_2_world.emplace_back(current_odometry);
            curr_tracker_status = TrackerStatus::VALID;
        }
        else
        {
            auto odometry_start = Timer::tic();

            cuda::ImageCuda<uchar, 3> c_global_color_map;
            cuda::ImageCuda<float, 3> c_global_vertices;
            cuda::ImageCuda<float, 3> c_global_normals;
            c_global_color_map.Upload(curr_global_color_map);
            c_global_vertices.Upload(curr_global_vertices);
            c_global_normals.Upload(curr_global_normals);

            cuda::RGBDOdometryCuda<3> odometry;
            odometry.SetIntrinsics(curr_frame.m_intrinsic);
            odometry.SetParameters(odometry::OdometryOption({ 20, 10, 5 }, 0.07), 0.5f, cuda::OdometryType::FRAME_TO_MODEL);
            spdlog::info("Initialize odometry");
            odometry.Initialize(source_rgbd, c_global_vertices, c_global_normals, c_global_color_map);

            auto result               = odometry.ComputeMultiScale();
            curr_relative_camera_pose = std::get<1>(result);
            auto success              = std::get<0>(result);

            if (success)
                curr_tracker_status = TrackerStatus::VALID;

            curr_information_matrix = odometry.ComputeInformationMatrix();
            auto odometry_time      = Timer::toc(odometry_start).count();
            spdlog::debug("Odometry took {} ms", odometry_time);
            current_odometry = mv_T_camera_2_world.back() * curr_relative_camera_pose;
            mv_T_camera_2_world.emplace_back(current_odometry);

            //! Is this required. It seems the destructor should automatically release resources
            c_global_color_map.Release();
            c_global_vertices.Release();
            c_global_normals.Release();
        }
        spdlog::debug("Current odometry: \n{}\n", current_odometry);

        return std::make_unique<TrackerOutput>(
            p_input->m_timestamp, curr_tracker_status, curr_frame, curr_relative_camera_pose, curr_information_matrix);
    }

    void Tracker::shutdown_queues()
    {
        MISOPipelineModule::shutdown_queues();
        m_frame_queue.shutdown();
    }
}  // namespace oslam
