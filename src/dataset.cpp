#include "dataset.h"

#include <Open3D/IO/ClassIO/ImageIO.h>
#include <boost/filesystem/operations.hpp>
#include <spdlog/spdlog.h>
#include <Open3D/Open3D.h>


oslam::RGBDdataset::RGBDdataset(const std::string &r_root_dir) : m_root_dir(r_root_dir), m_size(0)
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
        copy(fs::directory_iterator(mask_files_path),
          fs::directory_iterator(),
          std::back_inserter(m_mask_files));

        sort(m_rgb_files.begin(), m_rgb_files.end());
        sort(m_depth_files.begin(), m_depth_files.end());
        sort(m_mask_files.begin(), m_mask_files.end());

        spdlog::info("Total number of files: {}", m_rgb_files.size());
        m_size = m_rgb_files.size();

        if(fs::exists(intrinsic_path) && fs::is_regular_file(intrinsic_path)) {
            open3d::io::ReadIJsonConvertible(intrinsic_path.string(), intrinsic);
        }
    }
}

oslam::RGBDdataset::~RGBDdataset()
{
    // TODO: Delete allocations
}

oslam::RGBDdata oslam::RGBDdataset::get_data(unsigned int index)
{
    using namespace open3d::io;
    RGBDdata data;
    ReadImage(m_rgb_files.at(index).string(), data.color);
    ReadImage(m_depth_files.at(index).string(), data.depth);
    ReadImage(m_mask_files.at(index).string(), data.mask);
    data.gt_pose = Eigen::Matrix4d::Identity();

    return data;

}
