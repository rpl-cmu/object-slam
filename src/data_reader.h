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

        using FrameCallback    = std::function<void(Frame::Ptr)>;
        using ShutdownCallback = std::function<void(Timestamp)>;

        explicit DataReader(const std::string &root_dir);
        virtual ~DataReader() = default;

        //! \brief Runs through the dataset folder to read all files
        //!  - unless explicitly stopped
        //!  - all the files have been read
        //!
        bool run();

        //! \brief Shutdown the data_reader on demand
        void shutdown();

        //! \brief Register callbacks for output queues and shutdown notification
        inline void registerOutputCallback(const FrameCallback &r_callback) { output_callbacks_.emplace_back(r_callback); }
        inline void registerShutdownCallback(const ShutdownCallback &r_callback)
        {
            shutdown_callbacks_.emplace_back(r_callback);
        }

       private:
        bool parseDataset();
        bool readFrame();

        constexpr static int KEYFRAME_LENGTH = 10;
        fs::path root_dir_;                  //!< Root directory of the dataset folder
        std::size_t size_          = 0;      //!< Number of images color/depth in the dataset folder
        Timestamp curr_idx_        = 0;      //!< Current index of files being read
        bool dataset_parsed_       = false;  //!< is dataset parsed
        std::atomic_bool shutdown_ = false;  //!< Shutdown switch

        //! Vector of filepaths in the dataset folder
        std::vector<fs::path> rgb_files_;
        std::vector<fs::path> depth_files_;
        std::vector<fs::path> pose_files_;
        std::vector<fs::path> mask_files_;
        std::vector<fs::path> label_files_;

        open3d::camera::PinholeCameraIntrinsic intrinsic_;  //!< Camera intrinsics

        std::vector<FrameCallback> output_callbacks_;       //!< Called when data available
        std::vector<ShutdownCallback> shutdown_callbacks_;  //!< Forwards max num frames in dataset when shutdown
    };

} /* namespace oslam */
#endif /* OSLAM_DATA_READER_H */
