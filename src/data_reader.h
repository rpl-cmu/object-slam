/******************************************************************************
 * File:             data_reader.h
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      DataReader header: Reads the dataset from folder
 *****************************************************************************/
#ifndef OSLAM_DATA_READER_H
#define OSLAM_DATA_READER_H

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <Open3D/Open3D.h>

#include "utils/thread_safe_queue.h"

namespace oslam {

namespace fs = boost::filesystem;

struct RGBDdata
{
    open3d::camera::PinholeCameraIntrinsic intrinsic;
    open3d::geometry::Image color;
    open3d::geometry::Image depth;
    open3d::geometry::Image mask;
    Eigen::Matrix4d gt_pose;
    std::vector<unsigned int> labels;
    std::vector<float> scores;
};

/*! \class DataReader
 *  \brief Brief class description
 *
 *  Detailed description
 */
class DataReader
{
  public:
    typedef std::function<void(std::unique_ptr<RGBDdata>)> RGBDCallback;

    explicit DataReader(const std::string &r_root_dir);
    virtual ~DataReader() = default;

    void register_callback(const RGBDCallback &r_callback) { m_callbacks.push_back(r_callback); }

    bool run(void);

  protected:
    bool parse_dataset(void);
    bool read_frame(void);

    //! Root directory of the dataset folder
    fs::path m_root_dir;

    //! Number of images color/depth in the dataset folder
    std::size_t m_size = { 0 };
    //! Current index of files being read
    std::size_t m_current_index = 0;

    //! is dataset parsed
    bool m_dataset_parsed = { false };

    //! Vector of filepaths in the dataset folder
    std::vector<fs::path> m_rgb_files;
    std::vector<fs::path> m_depth_files;
    std::vector<fs::path> m_pose_files;
    std::vector<fs::path> m_mask_files;
    std::vector<fs::path> m_label_files;

    //! Camera intrinsics
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;

    //! Vector of callbacks called when data available
    std::vector<RGBDCallback> m_callbacks;

    //! Shutdown switch
    std::atomic_bool m_shutdown = { false };
};

} /* namespace oslam */
#endif /* OSLAM_DATA_READER_H */
