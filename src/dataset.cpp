/******************************************************************************
 * File:             dataset.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      Implementation for RGBDdataset
 *****************************************************************************/
#include "dataset.h"

#include <Open3D/Visualization/Utility/DrawGeometry.h>
#include <boost/filesystem/operations.hpp>
#include <functional>
#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>
#include <vector>

namespace oslam {

RGBDdataset::RGBDdataset(std::string root_dir) : m_root_dir(std::move(root_dir)), m_size(0)
{
    if (fs::exists(m_root_dir) && fs::is_directory(m_root_dir)) {
        fs::path color_files_path = m_root_dir / "color";
        fs::path depth_files_path = m_root_dir / "depth";
        fs::path mask_files_path = m_root_dir / "preprocessed";
        fs::path intrinsic_path = m_root_dir / "camera-intrinsics.json";

        spdlog::debug("{}, {}, {}",
          color_files_path.string(),
          depth_files_path.string(),
          mask_files_path.string());

        copy(fs::directory_iterator(color_files_path),
          fs::directory_iterator(),
          std::back_inserter(m_rgb_files));
        copy(fs::directory_iterator(depth_files_path),
          fs::directory_iterator(),
          std::back_inserter(m_depth_files));

        for (auto &file : fs::directory_iterator(mask_files_path)) {
            if (file.path().extension() == ".png")
                m_mask_files.push_back(file);
            else if (file.path().extension() == ".txt") {
                m_label_files.push_back(file);
            }
        }

        sort(m_rgb_files.begin(), m_rgb_files.end());
        sort(m_depth_files.begin(), m_depth_files.end());
        sort(m_mask_files.begin(), m_mask_files.end());

        spdlog::info("Total number of files: {}", m_rgb_files.size());
        m_size = m_rgb_files.size();

        if (fs::exists(intrinsic_path) && fs::is_regular_file(intrinsic_path)) {
            open3d::io::ReadIJsonConvertible(intrinsic_path.string(), intrinsic);
        }
    }
}

RGBDdata RGBDdataset::get_data(std::size_t index)
{
    using namespace open3d::io;
    RGBDdata data;
    spdlog::debug("Reading files: \n{} \n{} \n{}",
      m_rgb_files.at(index).string(),
      m_depth_files.at(index).string(),
      m_mask_files.at(index).string());
    ReadImage(m_rgb_files.at(index).string(), data.color);
    ReadImage(m_depth_files.at(index).string(), data.depth);
    ReadImage(m_mask_files.at(index).string(), data.mask);
    data.gt_pose = Eigen::Matrix4d::Identity();
    spdlog::debug("Image sizes: ({}, {}), ({}, {})",
      data.color.height_,
      data.color.width_,
      data.depth.height_,
      data.depth.width_);
    return data;
}

} // namespace oslam
