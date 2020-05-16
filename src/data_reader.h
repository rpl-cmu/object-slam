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
#include "frame.h"

namespace oslam {

namespace fs = boost::filesystem;

/*! \class DataReader
 *  \brief Reads from a given dataset folder given the following structure
 *  .
 *  ├── camera-intrinsics.json
 *  ├── color [723 entries]
 *  ├── depth [723 entries]
 *  └── preprocessed [1446 entries]
 *
 */
class DataReader
{
  public:
    OSLAM_POINTER_TYPEDEFS(DataReader);
    OSLAM_DELETE_COPY_CONSTRUCTORS(DataReader);

    typedef std::function<void(Frame::UniquePtr)> FrameCallback;

    explicit DataReader(const std::string &r_root_dir);
    virtual ~DataReader() = default;

    /*! \brief Runs through the dataset folder to read all files
     *  - unless explicitly stopped
     *  - all the files have been read
     */
    bool run();

    //! \brief Shutdown the data_reader on demand
    void shutdown(void);

    inline void register_provider_callback(const FrameCallback &r_callback)
    {
        m_provider_callback = r_callback;
    }

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
    FrameCallback m_provider_callback;

    //! Shutdown switch
    std::atomic_bool m_shutdown = { false };
};

} /* namespace oslam */
#endif /* OSLAM_DATA_READER_H */
