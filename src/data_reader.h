/******************************************************************************
 * File:             data_reader.h
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      DataReader header: Reads the dataset from folder
 *****************************************************************************/
#ifndef OSLAM_DATA_READER_H
#define OSLAM_DATA_READER_H

#include <Open3D/Open3D.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <string>
#include <vector>

#include "frame.h"
#include "utils/thread_safe_queue.h"

namespace oslam
{
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

        using FrameCallback    = std::function<void(const Frame::Ptr&)>;
        using ShutdownCallback = std::function<void(Timestamp)>;

        explicit DataReader(const std::string &r_root_dir);
        virtual ~DataReader() = default;

        /*! \brief Runs through the dataset folder to read all files
         *  - unless explicitly stopped
         *  - all the files have been read
         */
        bool run();

        //! \brief Shutdown the data_reader on demand
        void shutdown();

        //! \brief Register callbacks for output queues and shutdown notification
        inline void register_output_callback(const FrameCallback &r_callback)
        {
            m_output_callbacks.emplace_back(r_callback);
        }
        inline void register_shutdown_callback(const ShutdownCallback &r_callback)
        {
            m_shutdown_callbacks.emplace_back(r_callback);
        }

       private:
        bool parse_dataset();
        bool read_frame();

        //! Root directory of the dataset folder
        fs::path m_root_dir;

        //! Number of images color/depth in the dataset folder
        std::size_t m_size = { 0 };
        //! Current index of files being read
        Timestamp m_current_index = 0;

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
        std::vector<FrameCallback> m_output_callbacks;

        //! Vector of callbacks to forward max frames in dataset when shutdown
        std::vector<ShutdownCallback> m_shutdown_callbacks;

        //! Shutdown switch
        std::atomic_bool m_shutdown = { false };
    };

} /* namespace oslam */
#endif /* OSLAM_DATA_READER_H */
