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
#include <Cuda/Geometry/SegmentationCuda.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>

#include <cstdlib>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace oslam
{
    Tracker::Tracker(TransportOutputQueue *p_transport_output_queue, OutputQueue *p_output_queue)
        : MISO(p_output_queue, "Tracker"),
          m_frame_queue("InputFrameQueue"),
          mp_transport_output_queue(p_transport_output_queue),
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

        m_curr_timestamp                = p_input_frame->m_timestamp;
        const bool is_current_maskframe = p_input_frame->m_is_maskframe;

        Tracker::InputUniquePtr p_tracker_input;
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            p_tracker_input = std::make_unique<TrackerInput>(m_curr_timestamp, m_prev_maskframe_timestamp, *p_input_frame,
                                                             mp_prev_transport_output->m_instance_images);
        }
        else
        {
            ImageTransportOutput::UniquePtr p_transport_output;
            //! Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<ImageTransportOutput::UniquePtr>(m_curr_timestamp, mp_transport_output_queue,
                                                            &p_transport_output))
            {
                spdlog::error("Missing masked image with requested timestamp: {}", m_curr_timestamp);
                return nullptr;
            }
            if (!p_transport_output)
            {
                spdlog::error("Module: {} {} returned null", name_id_, mp_transport_output_queue->queue_id_);
                return nullptr;
            }
            p_tracker_input            = std::make_unique<TrackerInput>(m_curr_timestamp, m_curr_timestamp, *p_input_frame,
                                                             p_transport_output->m_instance_images);
            mp_prev_transport_output   = std::move(p_transport_output);
            m_prev_maskframe_timestamp = m_curr_timestamp;
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

        auto curr_frame               = p_input->m_frame;
        auto instance_images          = p_input->m_instance_images;
        auto prev_maskframe_timestamp = p_input->m_prev_maskframe_timestamp;
        spdlog::info("Previous maskframe timestamp: {}", prev_maskframe_timestamp);

        //! Upload current color and depth frame
        mc_curr_depth_raw.Upload(curr_frame.m_depth);
        mc_curr_color.Upload(curr_frame.m_color);

        //! Create vertex and normal maps from current frame
        cuda::RGBDImageCuda source_rgbd;
        source_rgbd.Build(mc_curr_depth_raw, mc_curr_color);

        if (m_curr_timestamp == 1)
        {
            auto current_odometry = Eigen::Matrix4d::Identity();

            //! Add Objects to the map and integrate with current_odometry
            {
                mr_global_map.add_background(curr_frame, current_odometry);
                unsigned int i = 0;
                for (auto &instance_image : instance_images)
                {
                    spdlog::debug("Current label, score: {}, {}", instance_image.m_label, instance_image.m_score);
                    mr_global_map.add_object(curr_frame, instance_image, current_odometry);
                    spdlog::info("Added object {}", instance_image.m_label);
                    i++;
                }
                spdlog::debug("Added all objects");
            }
            //! TODO(Akash): Make a callback to backend with current pose for optimisation
            mv_T_camera_2_world.emplace_back(current_odometry);

            mc_g_prev_color.Create(curr_frame.m_width, curr_frame.m_height);
            mc_g_prev_normal_map.Create(curr_frame.m_width, curr_frame.m_height);
            mc_g_prev_vertex_map.Create(curr_frame.m_width, curr_frame.m_height);
        }
        else
        {
            auto odometry_start = Timer::tic();

            cuda::RGBDOdometryCuda<3> odometry;
            odometry.SetIntrinsics(curr_frame.m_intrinsic);
            odometry.SetParameters(odometry::OdometryOption({ 20, 10, 5 }, 0.07), 0.5f, cuda::OdometryType::FRAME_TO_MODEL);

            odometry.Initialize(source_rgbd, mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color);

            auto result         = odometry.ComputeMultiScale();
            auto transformation = std::get<1>(result);

            //! TODO(Akash): Send this to backend optimizer
            /* auto information_matrix = odometry.ComputeInformationMatrix(); */

            auto odometry_time = Timer::toc(odometry_start).count();
            spdlog::debug("Odometry took {} ms", odometry_time);
            auto current_odometry = mv_T_camera_2_world.back() * transformation;
            mv_T_camera_2_world.emplace_back(current_odometry);
            auto integration_start = Timer::tic();

            mr_global_map.integrate_background(curr_frame, mv_T_camera_2_world.back());

            {
                //! Project the masks to current frame
                auto T_maskcamera_2_world             = mv_T_camera_2_world.at(prev_maskframe_timestamp - 1);
                auto T_camera_2_world                 = mv_T_camera_2_world.back();
                Eigen::Matrix4d T_maskcamera_2_camera = T_camera_2_world.inverse() * T_maskcamera_2_world;
                unsigned int i                        = 0;
                for (auto &instance_image : instance_images)
                {
                    mr_global_map.integrate_object(curr_frame, instance_image, mv_T_camera_2_world.back(),
                                                   T_maskcamera_2_camera);
                    i++;
                }
            }

            auto integration_time = Timer::toc(integration_start).count();
            spdlog::debug("Integration took {} ms", integration_time);
        }
        spdlog::info("Number of objects in map: {}", mr_global_map.size());

        auto raycast_start = Timer::tic();

        mr_global_map.raycast_background(mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color,
                                         mv_T_camera_2_world.back());

        auto raycast_time = Timer::toc(raycast_start).count();
        spdlog::debug("Raycast took {} ms", raycast_time);

        cv::Mat color_map  = mc_g_prev_color.DownloadMat();

        cv::imshow("Color map", color_map);
        cv::imshow("Source frame", curr_frame.m_color);
        cv::waitKey();

        return nullptr;
    }

    void Tracker::shutdown_queues()
    {
        m_frame_queue.shutdown();
        MISOPipelineModule::shutdown_queues();
    }
}  // namespace oslam
