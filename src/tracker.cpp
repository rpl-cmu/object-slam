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
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "cuda/segmentation.cuh"

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
      p_tracker_payload = std::make_unique<TrackerPayload>(m_curr_timestamp, *p_input_frame, *mp_prev_masked_image);
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
      p_tracker_payload    = std::make_unique<TrackerPayload>(m_curr_timestamp, *p_input_frame, *p_masked_image);
      mp_prev_masked_image = std::move(p_masked_image);
    }
    if (!p_tracker_payload)
    {
      spdlog::error("Unable to create TrackerPayload");
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

    auto curr_frame       = p_input->get_frame();
    auto curr_depth_image = p_input->get_frame().get_depth();
    auto curr_color_image = p_input->get_frame().get_color();

    auto curr_masked_image = p_input->get_masked_image();
    auto mat_masks         = curr_masked_image.process();

    mc_curr_depth_raw.Upload(curr_depth_image);
    mc_curr_color.Upload(curr_color_image);

    //! First ever frame
    if (m_curr_timestamp == 1)
    {
      auto current_odometry = Eigen::Matrix4d::Identity();
      //! Add background object to map and integrate with current camera pose
      mr_global_map.add_background(curr_frame, current_odometry);
      spdlog::info("Add background complete");
      /* for(unsigned int i = 0; auto& mask : mat_masks) */
      /* { */
      /*     auto label = curr_masked_image.m_labels[i]; */
      /*     auto score = curr_masked_image.m_scores[i]; */
      /*     mr_global_map.add_object(mc_curr_color, mc_curr_depth_raw, mask, label, score,
       * mc_intrinsic, mv_T_camera_2_world.back()); */
      /*     i++; */
      /* } */

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

      cuda::RGBDImageCuda source_rgbd;
      source_rgbd.Build(mc_curr_depth_raw, mc_curr_color);

      odometry.Initialize(source_rgbd, mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color);

      auto result         = odometry.ComputeMultiScale();
      auto transformation = std::get<1>(result);

      //! TODO(Akash): Send this to backend optimizer
      /* auto information_matrix = odometry.ComputeInformationMatrix(); */

      auto odometry_time = Timer::toc(odometry_start).count();
      spdlog::debug("Odometry took {} ms", odometry_time);
      auto current_odometry = mv_T_camera_2_world.back() * transformation;
      mv_T_camera_2_world.emplace_back(current_odometry);
    }

    auto integration_start = Timer::tic();

    mr_global_map.integrate_background(curr_frame, mv_T_camera_2_world.back());

    auto integration_time = Timer::toc(integration_start).count();
    spdlog::debug("Integration took {} ms", integration_time);

    auto raycast_start = Timer::tic();

    mr_global_map.raycast_background(mc_g_prev_vertex_map, mc_g_prev_normal_map, mc_g_prev_color,
                                     mv_T_camera_2_world.back());

    auto raycast_time = Timer::toc(raycast_start).count();
    spdlog::debug("Raycast took {} ms", raycast_time);

    std::shared_ptr<geometry::Image> p_vertex_map = mc_g_prev_vertex_map.DownloadImage();
    std::shared_ptr<geometry::Image> p_normal_map = mc_g_prev_normal_map.DownloadImage();
    std::shared_ptr<geometry::Image> p_color_map  = mc_g_prev_color.DownloadImage();

    cv::Mat vertex_map(p_vertex_map->height_, p_vertex_map->width_, CV_32FC3, p_vertex_map->data_.data());
    cv::Mat normal_map(p_normal_map->height_, p_normal_map->width_, CV_32FC3, p_normal_map->data_.data());
    cv::Mat color_image(p_color_map->height_, p_color_map->width_, CV_8UC3, p_color_map->data_.data());
    cv::Mat curr_image(curr_frame.get_color().height_, curr_frame.get_color().width_, CV_8UC3,
                       curr_frame.get_color().data_.data());
    cv::Mat output_curr_image;
    cv::cvtColor(curr_image, output_curr_image, cv::COLOR_RGB2BGR);

    cv::Mat output_color_image;
    cv::cvtColor(color_image, output_color_image, cv::COLOR_RGB2BGR);
    cv::imshow("Vertex map", vertex_map);
    cv::imshow("Normal map", normal_map);
    cv::imshow("Color map", output_color_image);
    cv::imshow("Curent source frame", output_curr_image);
    cv::waitKey(1);

    return nullptr;

    /* cuda::ImageCuda<float, 1> c_edge_map = */
    /*  cuda::SegmentationCuda::ComputeEdgeMap(mc_curr_vertex_map, mc_curr_normal_map); */

    /* //! TODO: Combine the masked image and geometric segmentation to obtain mask */
    /* //! Process masks to obtain list of object frames (frames, with masked depths) */
    /* //! Instantiate objects */

    /* std::shared_ptr<geometry::Image> p_depth = mc_curr_depth_filt.DownloadImage(); */
    /* std::shared_ptr<geometry::Image> p_vertex_map = mc_curr_vertex_map.DownloadImage(); */
    /* std::shared_ptr<geometry::Image> p_normal_map = mc_curr_normal_map.DownloadImage(); */
    /* std::shared_ptr<geometry::Image> p_edge_map   = c_edge_map.DownloadImage(); */

    /* /1* spdlog::info("Bilateral filter took: {} ms", Timer::toc(start_time).count()); *1/ */
    /* cv::Mat download_image(p_depth->height_, p_depth->width_, CV_32FC1, p_depth->data_.data()); */
    /* cv::Mat vertex_map(p_vertex_map->height_, p_vertex_map->width_, CV_32FC3, p_vertex_map->data_.data()); */
    /* cv::Mat normal_map(p_normal_map->height_, p_normal_map->width_, CV_32FC3, p_normal_map->data_.data()); */
    /* cv::Mat edge_map(p_edge_map->height_, p_edge_map->width_, CV_32FC1, p_edge_map->data_.data()); */

    /* cv::Mat binary_edge_map(p_edge_map->height_, p_edge_map->width_, CV_8UC1); */
    /* cv::threshold(edge_map, binary_edge_map, 0.3, 255, cv::THRESH_BINARY_INV); */

    /* cv::Mat labels_image, binary_int; */
    /* binary_edge_map.convertTo(binary_int, CV_8U); */
    /* cv::connectedComponents(binary_int, labels_image, 4); */

    /* const unsigned char colors[31][3] = { */
    /*     {0, 0, 0},     {0, 0, 255},     {255, 0, 0},   {0, 255, 0},     {255, 26, 184},  {255,211, 0},   {0, 131, 246},
     * {0, 140, 70}, */
    /*     {167, 96, 61}, {79, 0, 105},    {0, 255, 246}, {61, 123, 140},  {237, 167, 255}, {211, 255, 149}, {184, 79, 255},
     * {228, 26, 87}, */
    /*     {131, 131, 0}, {0, 255, 149},   {96, 0, 43},   {246, 131, 17},  {202, 255, 0},   {43, 61, 0},     {0, 52, 193},
     * {255, 202, 131}, */
    /*     {0, 43, 96},   {158, 114, 140}, {79, 184, 17}, {158, 193, 255}, {149, 158, 123}, {255, 123, 175}, {158, 8, 0}}; */
    /* auto getColor = [&colors](unsigned index) -> cv::Vec3b { */
    /*     return (index == 255) ? cv::Vec3b(255, 255, 255) : (cv::Vec3b)colors[index % 31]; */
    /* }; */

    /* auto mapLabelToColorImage = [&getColor](cv::Mat input, bool white0 = false) -> cv::Mat { */
    /*     std::function<cv::Vec3b(unsigned)> getIndex; */
    /*     auto getColorWW = [&](unsigned index) -> cv::Vec3b { return (white0 && index == 0) ? cv::Vec3b(255, 255, 255) :
     * getColor(index); }; */
    /*     if (input.type() == CV_32SC1) */
    /*         getIndex = [&](unsigned i) -> cv::Vec3b { return getColorWW(input.at<int>(i)); }; */
    /*     else if (input.type() == CV_8UC1) */
    /*         getIndex = [&](unsigned i) -> cv::Vec3b { return getColorWW(input.data[i]); }; */
    /*     else */
    /*         assert(0); */
    /*     cv::Mat result(input.rows, input.cols, CV_8UC3); */
    /*     for (unsigned i = 0; i < result.total(); ++i) { */
    /*         ((cv::Vec3b*)result.data)[i] = getIndex(i); */
    /*     } */
    /*     return result; */
    /* }; */

    /* cv::imshow("Connected components", mapLabelToColorImage(labels_image)); */
    /* cv::imshow("Bilateral", download_image); */
    /* cv::imshow("Vertex map", vertex_map); */
    /* cv::imshow("Normal map", normal_map); */
    /* cv::imshow("Edge map", binary_edge_map); */
    /* cv::waitKey(2); */

    /* return nullptr; */
  }

  void Tracker::shutdown_queues()
  {
    m_frame_queue.shutdown();
    MISOPipelineModule::shutdown_queues();
  }
}  // namespace oslam
