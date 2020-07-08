/******************************************************************************
 * File:             mapper.cpp
 *
 * Author:           Akash Sharma
 * Created:          06/26/20
 * Description:      Mapper thread implementation
 *****************************************************************************/
#include "mapper.h"

#include <Cuda/OSLAMUtils/SegmentationCuda.h>

#include <memory>
#include <opencv2/rgbd/depth.hpp>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    Mapper::Mapper(
        TrackerOutputQueue* p_tracker_output_queue,
        TransportOutputQueue* p_transport_output_queue,
        OutputQueue* p_output_queue)
        : MISO(p_output_queue, "Mapper"),
          mp_tracker_output_queue(p_tracker_output_queue),
          mp_transport_output_queue(p_transport_output_queue),
          mr_global_map(GlobalMap::get_instance())
    {
    }

    Mapper::InputUniquePtr Mapper::get_input_packet()
    {
        auto start_time = Timer::tic();

        TrackerOutput::UniquePtr p_mapper_payload;
        bool queue_state = mp_tracker_output_queue->popBlocking(p_mapper_payload);
        if (!p_mapper_payload || !queue_state)
        {
            spdlog::error("Module: {} {} returned null", name_id_, mp_tracker_output_queue->queue_id_);
        }

        m_curr_timestamp                = p_mapper_payload->m_timestamp;
        const bool is_current_maskframe = p_mapper_payload->m_frame.m_is_maskframe;

        Mapper::InputUniquePtr p_mapper_input;
        if (!is_current_maskframe)
        {
            spdlog::debug("Use old mask and geometric segment");
            //! Simply use the previous masked image to create tracker payload
            p_mapper_input = std::make_unique<MapperInput>(
                m_curr_timestamp,
                m_prev_maskframe_timestamp,
                p_mapper_payload->m_tracker_status,
                p_mapper_payload->m_frame,
                mp_prev_transport_output->m_instance_images,
                p_mapper_payload->m_relative_camera_pose,
                p_mapper_payload->m_information_matrix);
        }
        else
        {
            ImageTransportOutput::UniquePtr p_transport_output;
            //! Try to synchronize the ImageTransportOutputQueue and search for image with same timestamp as
            //! current frame
            if (!syncQueue<ImageTransportOutput::UniquePtr>(
                    m_curr_timestamp, mp_transport_output_queue, &p_transport_output))
            {
                spdlog::error("Missing masked image with requested timestamp: {}", m_curr_timestamp);
                return nullptr;
            }
            if (!p_transport_output)
            {
                spdlog::error("Module: {} {} returned null", name_id_, mp_transport_output_queue->queue_id_);
                return nullptr;
            }
            p_mapper_input = std::make_unique<MapperInput>(
                m_curr_timestamp,
                m_curr_timestamp,
                p_mapper_payload->m_tracker_status,
                p_mapper_payload->m_frame,
                p_transport_output->m_instance_images,
                p_mapper_payload->m_relative_camera_pose,
                p_mapper_payload->m_information_matrix);

            mp_prev_transport_output   = std::move(p_transport_output);
            m_prev_maskframe_timestamp = m_curr_timestamp;
        }
        if (!p_mapper_input)
        {
            spdlog::error("Unable to create MapperInput");
            return nullptr;
        }
        auto duration = Timer::toc(start_time).count();
        spdlog::info("Processed Mapper payload: {}, took {} ms", m_curr_timestamp, duration);

        return p_mapper_input;
    }

    Mapper::OutputUniquePtr Mapper::run_once(Mapper::InputUniquePtr p_input)
    {
        MapperInput& mapper_payload                      = *p_input;
        m_curr_timestamp                                 = mapper_payload.m_timestamp;
        const Timestamp prev_maskframe_timestamp         = mapper_payload.m_prev_maskframe_timestamp;
        const Frame& curr_frame                          = mapper_payload.m_frame;
        const InstanceImages& curr_instance_images       = mapper_payload.m_instance_images;
        const Eigen::Matrix4d& curr_relative_camera_pose = mapper_payload.m_relative_camera_pose;

        MapperStatus curr_mapper_status = MapperStatus::INVALID;
        if (m_curr_timestamp == 1)
        {
            // Instantiate objects and background in map
            if (mapper_payload.m_tracker_status == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d curr_camera_pose = curr_relative_camera_pose;
                mv_T_camera_2_world.emplace_back(curr_camera_pose);

                //! Add camera pose and factor to the graph
                auto background_pair = mr_global_map.create_background(curr_frame, curr_camera_pose);

                //! Add each object in the first frame as a landmark
                for (const auto& instance_image : curr_instance_images)
                {
                    auto object_pair =
                        mr_global_map.create_object(curr_frame, instance_image, mapper_payload.m_relative_camera_pose);
                }
                curr_mapper_status = MapperStatus::VALID;
            }
        }
        else
        {
            if (mapper_payload.m_tracker_status == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d curr_camera_pose = mv_T_camera_2_world.back() * curr_relative_camera_pose;
                mv_T_camera_2_world.emplace_back(curr_camera_pose);

                mr_global_map.integrate_background(curr_frame, mv_T_camera_2_world.back());

                //! Project the masks to current frame
                auto T_maskcamera_2_world             = mv_T_camera_2_world.at(prev_maskframe_timestamp - 1);
                auto T_camera_2_world                 = mv_T_camera_2_world.back();
                Eigen::Matrix4d T_maskcamera_2_camera = T_camera_2_world.inverse() * T_maskcamera_2_world;
                unsigned int i                        = 0;

                cuda::TransformCuda transform_maskcamera_to_camera;
                transform_maskcamera_to_camera.FromEigen(T_maskcamera_2_camera);
                cuda::PinholeCameraIntrinsicCuda intrinsic(curr_frame.m_intrinsic);
                for (auto& instance_image : curr_instance_images)
                {
                    //! Skip for background object
                    if(instance_image.m_label == 0)
                        continue;
                    spdlog::info("Processing {}th instance image in payload: {}", i, m_curr_timestamp);
                    //! Project the mask from maskframe to current frame
                    /* cv::imshow("Input instance image mask", instance_image.m_maskb); */
                    cuda::ImageCuda<uchar, 1> src_mask;
                    src_mask.Upload(instance_image.m_maskb);
                    cv::Mat depthf;
                    cv::rgbd::rescaleDepth(curr_frame.m_depth, CV_32F, depthf);
                    cuda::ImageCuda<float, 1> depth;
                    depth.Upload(depthf);
                    auto c_proj_mask = cuda::SegmentationCuda::TransformAndProject(
                        src_mask, depth, transform_maskcamera_to_camera, intrinsic);
                    cv::Mat src_proj_mask = c_proj_mask.DownloadMat();
                    BoundingBox src_proj_bbox =
                        transform_project_bbox(instance_image.m_bbox, depthf, curr_frame.m_intrinsic, T_maskcamera_2_camera);
                    /* cv::imshow("Source projected mask", src_proj_mask); */

                    InstanceImage proj_instance_image(
                        src_proj_mask, src_proj_bbox, instance_image.m_label, instance_image.m_score);
                    depth.Release();

                    bool matched =
                        mr_global_map.integrate_object(curr_frame, proj_instance_image, mv_T_camera_2_world.back());

                    if (!matched)
                        mr_global_map.create_object(curr_frame, proj_instance_image, mv_T_camera_2_world.back());
                    i++;
                }
                curr_mapper_status = MapperStatus::VALID;
            }
            // TODO: Delete objects if required
        }

        return std::make_unique<RendererInput>(m_curr_timestamp, curr_mapper_status, curr_frame, mv_T_camera_2_world.back());
    }
}  // namespace oslam
