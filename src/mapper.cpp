/******************************************************************************
 * File:             mapper.cpp
 *
 * Author:           Akash Sharma
 * Created:          06/26/20
 * Description:      Mapper thread implementation
 *****************************************************************************/
#include "mapper.h"

#include <Cuda/OSLAMUtils/SegmentationCuda.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <memory>
#include <opencv2/rgbd/depth.hpp>

#include "instance_image.h"
#include "utils/utils.h"

namespace oslam
{
    Mapper::Mapper(InputQueue* p_input_queue, OutputQueue* p_output_queue)
        : SISO(p_input_queue, p_output_queue, "Mapper"), mr_global_map(GlobalMap::get_instance())
    {
    }

    Mapper::OutputUniquePtr Mapper::run_once(Mapper::InputUniquePtr p_input)
    {
        using namespace gtsam;
        m_curr_timestamp                                 = p_input->m_timestamp;
        TrackerOutput& tracker_payload                   = *p_input;
        const Timestamp prev_maskframe_timestamp         = tracker_payload.m_prev_maskframe_timestamp;
        const Frame& curr_frame                          = tracker_payload.m_frame;
        const InstanceImages& curr_instance_images       = tracker_payload.m_instance_images;
        const Eigen::Matrix4d& curr_relative_camera_pose = tracker_payload.m_relative_camera_pose;

        if (m_curr_timestamp == 1)
        {
            // Instantiate objects and background in map
            if (tracker_payload.m_tracker_status == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d curr_camera_pose = curr_relative_camera_pose;
                mv_T_camera_2_world.emplace_back(curr_camera_pose);

                //! Add camera pose and factor to the graph
                auto background_pair = mr_global_map.create_background(curr_frame, curr_camera_pose);

                //! Add camera measurement as prior
                Key camera_key                          = gtsam::Symbol('c', tracker_payload.m_timestamp);
                Eigen::Vector6d precisions              = Eigen::Vector6d::Constant(0.01);
                const noiseModel::Diagonal::shared_ptr& noise = noiseModel::Diagonal::Precisions(precisions);
                //! Not sure if there needs to be a prior factor for camera
                m_factor_graph.push_back(
                    boost::make_shared<PriorFactor<Pose3>>(camera_key, gtsam::Pose3(curr_camera_pose), noise));

                //! Add background to pose graph
                Key background_key = gtsam::Symbol('o', std::hash<ObjectId>()(background_pair.first));
                //! TODO: Map object needs a way of referring to global objects in map
                m_factor_graph.push_back(boost::make_shared <
                                         PriorFactor<Pose3>>(background_key, gtsam::Pose3(background_pair.second->get_pose()), noise));
                m_factor_graph.push_back(boost::make_shared<BetweenFactor<Pose3>>(camera_key, background_key, gtsam::Pose3(Eigen::Matrix4d::Identity()), noise));

                //! Add each object in the first frame as a landmark
                for (const auto& instance_image : curr_instance_images)
                {
                    auto object_pair=
                        mr_global_map.create_object(curr_frame, instance_image, tracker_payload.m_relative_camera_pose);
                    Key object_key = gtsam::Symbol('o', std::hash<ObjectId>()(object_pair.first));

                    Eigen::Matrix4d T_camera_2_object = object_pair.second->get_pose().inverse() * curr_camera_pose;
                    m_factor_graph.push_back(boost::make_shared<BetweenFactor<Pose3>>(camera_key, object_key, gtsam::Pose3(T_camera_2_object), noise));
                }
            }
        }
        else
        {
            // Associate frame to objects in map
            if (tracker_payload.m_tracker_status == TrackerStatus::VALID)
            {
                const Eigen::Matrix4d curr_camera_pose = mv_T_camera_2_world.back() * curr_relative_camera_pose;
                mv_T_camera_2_world.emplace_back(curr_camera_pose);

                const Eigen::Matrix4d& T_maskcamera_2_world = mv_T_camera_2_world.at(prev_maskframe_timestamp - 1);
                const Eigen::Matrix4d& T_camera_2_world = mv_T_camera_2_world.back();
                Eigen::Matrix4d T_maskcamera_2_camera = T_camera_2_world.inverse() * T_maskcamera_2_world;

                cuda::TransformCuda transform_maskcamera_to_camera;
                transform_maskcamera_to_camera.FromEigen(T_maskcamera_2_camera);
                cuda::PinholeCameraIntrinsicCuda intrinsic(curr_frame.m_intrinsic);

                for (const auto& instance_image : curr_instance_images)
                {
                    //! Project the mask from maskframe to current frame
                    cuda::ImageCuda<uchar, 1> src_mask;
                    src_mask.Upload(instance_image.m_maskb);
                    cv::Mat depthf;
                    cv::rgbd::rescaleDepth(curr_frame.m_depth, CV_32F, depthf);
                    cuda::ImageCuda<float, 1> depth;
                    depth.Upload(depthf);
                    auto c_proj_mask =
                        cuda::SegmentationCuda::TransformAndProject(src_mask, depth, transform_maskcamera_to_camera, intrinsic);
                    cv::Mat src_proj_mask = c_proj_mask.DownloadMat();
                    BoundingBox src_proj_bbox =
                        transform_project_bbox(instance_image.m_bbox, depthf, curr_frame.m_intrinsic, T_maskcamera_2_camera);
                    cv::imshow("Source projected mask", src_proj_mask);

                    InstanceImage proj_instance_image(src_proj_mask, src_proj_bbox, instance_image.m_label, instance_image.m_score);
                    depth.Release();

                    //! Integrate the object with projected instance image
                    bool matched = mr_global_map.integrate_object(curr_frame, proj_instance_image, curr_camera_pose);

                    if(!matched)
                    {
                        mr_global_map.create_object(curr_frame, proj_instance_image, curr_camera_pose);

                        //! Add object to camera factors for current camera factor
                    }
                }

                //! Every 50 frames create a camera pose to optimize and instantiate new background
                if(m_curr_timestamp % 50 == 0)
                {
                }

            }
            // Delete objects if required
        }
    }
}  // namespace oslam
