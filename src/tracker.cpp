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
    Tracker::Tracker(MaskedImageQueue *p_masked_image_queue, OutputQueue *p_output_queue)
        : MISO(p_output_queue, "Tracker"),
          m_frame_queue("InputFrameQueue"),
          mp_masked_image_queue(p_masked_image_queue),
          mr_global_map(GlobalMap::get_instance())
    {
    }

    Tracker::InputUniquePtr Tracker::get_input_packet()
    {
        auto start_time = Timer::tic();

        Frame::UniquePtr p_input_frame;
        bool queue_state = false;
        queue_state      = m_frame_queue.popBlocking(p_input_frame);

        if (!queue_state)
        {
            spdlog::error("Module: {} {} is down", name_id_, mp_masked_image_queue->queue_id_);
            return nullptr;
        }
        if (!p_input_frame)
        {
            spdlog::error("Module: {} {} returned null", name_id_, mp_masked_image_queue->queue_id_);
        }
        m_curr_timestamp                = p_input_frame->m_timestamp;
        const bool is_current_maskframe = p_input_frame->is_maskframe();

        Tracker::InputUniquePtr p_tracker_payload;
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            p_tracker_payload = std::make_unique<TrackerInputPayload>(m_curr_timestamp, m_prev_maskframe_timestamp,
                                                                      *p_input_frame, *mp_prev_masked_image);
        }
        else
        {
            MaskedImage::UniquePtr p_masked_image;

            //! Try to synchronize the Masked Image queue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<MaskedImage::UniquePtr>(m_curr_timestamp, mp_masked_image_queue, &p_masked_image))
            {
                spdlog::error("Missing masked image with requested timestamp: {}", m_curr_timestamp);
                return nullptr;
            }
            if (!p_masked_image)
            {
                spdlog::error("Module: {} {} returned null", name_id_, mp_masked_image_queue->queue_id_);
                return nullptr;
            }
            p_tracker_payload =
                std::make_unique<TrackerInputPayload>(m_curr_timestamp, m_curr_timestamp, *p_input_frame, *p_masked_image);
            mp_prev_masked_image       = std::move(p_masked_image);
            m_prev_maskframe_timestamp = m_curr_timestamp;
        }
        if (!p_tracker_payload)
        {
            spdlog::error("Unable to create TrackerInputPayload");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::info("Processed tracker payload: {}, took {} ms", m_curr_timestamp, duration);

        return p_tracker_payload;
    }

    Tracker::OutputUniquePtr Tracker::run_once(Tracker::InputUniquePtr p_input)
    {
        using namespace open3d;
        if (m_first_run)
        {
            CheckCuda(cudaSetDevice(0));
            m_first_run = false;
            camera::PinholeCameraIntrinsic intrinsic(p_input->get_frame().get_intrinsics());
            mc_intrinsic.SetIntrinsics(intrinsic);
        }

        auto curr_frame               = p_input->get_frame();
        auto curr_masked_image        = p_input->get_masked_image();
        auto prev_maskframe_timestamp = p_input->get_prev_maskframe_timestamp();
        spdlog::info("Previous maskframe timestamp: {}", prev_maskframe_timestamp);

        auto curr_depth_image = curr_frame.get_depth();
        auto curr_color_image = curr_frame.get_color();
        auto mat_masks        = curr_masked_image.process();

        mc_curr_depth_raw.Upload(curr_depth_image);
        mc_curr_color.Upload(curr_color_image);

        cuda::RGBDImageCuda source_rgbd;
        source_rgbd.Build(mc_curr_depth_raw, mc_curr_color);
        auto curr_vertex_map = source_rgbd.depth_.GetVertexMap(mc_intrinsic);
        auto curr_normal_map = curr_vertex_map.GetNormalMap();

        //! First ever frame
        if (m_curr_timestamp == 1)
        {
            auto current_odometry = Eigen::Matrix4d::Identity();

            //! Add Objects to the map and integrate with current_odometry
            {
                mr_global_map.add_background(curr_frame, current_odometry);
                unsigned int i = 0;
                for (auto &mask : mat_masks)
                {
                    auto label = curr_masked_image.m_labels[i];
                    auto score = curr_masked_image.m_scores[i];
                    spdlog::debug("Current label, score: {}, {}", label, score);
                    mr_global_map.add_object(curr_frame, mask, label, score, current_odometry);
                    i++;
                }
                spdlog::debug("Added all objects");
            }
            //! TODO(Akash): Make a callback to backend with current pose for optimisation
            mv_T_camera_2_world.emplace_back(current_odometry);

            mc_g_prev_color.Create(curr_color_image.width_, curr_color_image.height_);
            mc_g_prev_normal_map.Create(curr_color_image.width_, curr_color_image.height_);
            mc_g_prev_vertex_map.Create(curr_color_image.width_, curr_color_image.height_);
        }
        else
        {
            auto odometry_start = Timer::tic();

            cuda::RGBDOdometryCuda<3> odometry;
            odometry.SetIntrinsics(p_input->get_frame().get_intrinsics());
            odometry.SetParameters(odometry::OdometryOption({ 20, 10, 5 }, 0.07), 0.5f, cuda::OdometryType::FRAME_TO_MODEL);

            odometry.Initialize(source_rgbd, mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color);

            curr_vertex_map = odometry.source_vertex_[0];
            curr_normal_map = odometry.source_normal_[0];
            auto result     = odometry.ComputeMultiScale();
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
                for (auto &mask : mat_masks)
                {
                    auto label = curr_masked_image.m_labels[i];
                    auto score = curr_masked_image.m_scores[i];
                    mr_global_map.integrate_object(curr_frame, mask, label, score, mv_T_camera_2_world.back(),
                                                   T_maskcamera_2_camera);
                    spdlog::debug("Integrated object: {}", i);
                    i++;
                }
            }

            auto integration_time = Timer::toc(integration_start).count();
            spdlog::debug("Integration took {} ms", integration_time);
        }

        auto raycast_start = Timer::tic();

        mr_global_map.raycast_background(mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color,
                                         mv_T_camera_2_world.back());

        auto raycast_time = Timer::toc(raycast_start).count();
        spdlog::debug("Raycast took {} ms", raycast_time);

        cv::Mat vertex_map = mc_g_prev_vertex_map.DownloadMat();
        cv::Mat normal_map = mc_g_prev_normal_map.DownloadMat();
        cv::Mat color_map  = mc_g_prev_color.DownloadMat();
        cv::Mat curr_image = curr_frame.get_color_mat();

        cv::Mat output_color_image;
        cv::cvtColor(color_map, output_color_image, cv::COLOR_RGB2BGR);

        cv::imshow("Color map", output_color_image);
        cv::imshow("Source frame", curr_image);
        cv::waitKey(1);

        return nullptr;
    }

    void Tracker::shutdown_queues()
    {
        m_frame_queue.shutdown();
        MISOPipelineModule::shutdown_queues();
    }
}  // namespace oslam
