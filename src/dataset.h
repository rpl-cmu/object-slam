/******************************************************************************
 * File:             dataset.h
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      RGBDdataset header: Reads the dataset from folder
 *****************************************************************************/
#ifndef OSLAM_DATASET_H
#define OSLAM_DATASET_H

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <Open3D/Open3D.h>

namespace oslam {

namespace fs = boost::filesystem;

struct RGBDdata
{
    open3d::geometry::Image color;
    open3d::geometry::Image depth;
    open3d::geometry::Image mask;
    Eigen::Matrix4d gt_pose;
    std::vector<unsigned int> labels;
    std::vector<float> scores;
};

class RGBDdataset
{
  public:
    explicit RGBDdataset(std::string root_dir);
    virtual ~RGBDdataset();

    inline std::size_t size(void) const { return m_size; }
    RGBDdata get_data(std::size_t index);

    open3d::camera::PinholeCameraIntrinsic intrinsic;

  private:
    fs::path m_root_dir;
    std::size_t m_size;
    std::vector<fs::path> m_rgb_files;
    std::vector<fs::path> m_depth_files;
    std::vector<fs::path> m_pose_files;
    std::vector<fs::path> m_mask_files;
    std::vector<fs::path> m_label_files;
};

} /* namespace oslam */
#endif /* OSLAM_DATASET_H */
