/******************************************************************************
 * File:             dataset.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Implementation for RGBDdataset
 *****************************************************************************/
#include "data_reader.h"

#include <Open3D/Open3D.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <spdlog/spdlog.h>

#include <boost/filesystem/operations.hpp>
#include <functional>
#include <memory>
#include <vector>

namespace oslam
{
  DataReader::DataReader(const std::string &r_root_dir) : m_root_dir(r_root_dir) {}

  bool DataReader::run()
  {
    if (!m_shutdown && !m_dataset_parsed)
    {
      m_dataset_parsed = parse_dataset();
    }

    if (m_dataset_parsed)
    {
      while (!m_shutdown && read_frame())
      {
        ++m_current_index;
      }
    }

    if (m_shutdown)
    {
      spdlog::info("DataReader Shutdown requested");
    }
    return false;
  }

  bool DataReader::parse_dataset()
  {
    if (!fs::exists(m_root_dir) || !fs::is_directory(m_root_dir))
    {
      spdlog::error("Incorrect dataset path");
    }

    fs::path color_files_path = m_root_dir / "color";
    fs::path depth_files_path = m_root_dir / "depth";
    fs::path mask_files_path  = m_root_dir / "preprocessed";
    fs::path intrinsic_path   = m_root_dir / "camera-intrinsics.json";

    spdlog::debug("{}, {}, {}", color_files_path.string(), depth_files_path.string(), mask_files_path.string());

    copy(fs::directory_iterator(color_files_path), fs::directory_iterator(), std::back_inserter(m_rgb_files));
    copy(fs::directory_iterator(depth_files_path), fs::directory_iterator(), std::back_inserter(m_depth_files));

    for (auto &file : fs::directory_iterator(mask_files_path))
    {
      if (file.path().extension() == ".png")
      {
        m_mask_files.push_back(file);
      }
      else if (file.path().extension() == ".txt")
      {
        m_label_files.push_back(file);
      }
      else
      {
        spdlog::error("Groundtruth segmentation has incorrect extension", file.path());
      }
    }

    sort(m_rgb_files.begin(), m_rgb_files.end());
    sort(m_depth_files.begin(), m_depth_files.end());
    sort(m_mask_files.begin(), m_mask_files.end());

    if (m_rgb_files.size() != m_depth_files.size())
    {
      spdlog::error("Number of Color images and Depth images do not match");
    }

    if (!fs::exists(intrinsic_path) || !fs::is_regular_file(intrinsic_path))
    {
      spdlog::error("Could not find camera intrinsics");
    }

    open3d::io::ReadIJsonConvertible(intrinsic_path.string(), m_intrinsic);

    m_size = m_rgb_files.size();
    spdlog::info("Total number of files: {}", m_rgb_files.size());
    return true;
  }

  bool DataReader::read_frame()
  {
    if (m_current_index >= m_size)
    {
      for (const auto &callback : m_shutdown_callbacks)
      {
        callback(m_current_index);
      }
      return false;
    }

    using namespace open3d::io;

    spdlog::debug("Reading files: \n{} \n{} \n{}", m_rgb_files.at(m_current_index).string(),
                  m_depth_files.at(m_current_index).string(), m_mask_files.at(m_current_index).string());

    open3d::geometry::Image color;
    open3d::geometry::Image depth;
    open3d::geometry::Image gt_mask;
    ReadImage(m_rgb_files.at(m_current_index).string(), color);
    ReadImage(m_depth_files.at(m_current_index).string(), depth);
    ReadImage(m_mask_files.at(m_current_index).string(), gt_mask);

    // TODO:(Akash) Read ground truth pose from file conditionally based on input parameter
    /* p_data->gt_pose = Eigen::Matrix4d::Identity(); */

    // Every 10th frame requires semantic segmentation
    m_provider_callback(
        std::make_unique<Frame>(m_current_index + 1, m_intrinsic, color, depth, (m_current_index % 10) == 0));
    return true;
  }

  void DataReader::shutdown()
  {
    spdlog::info("Shutting down DataReader on demand");
    m_shutdown = true;
  }

}  // namespace oslam
