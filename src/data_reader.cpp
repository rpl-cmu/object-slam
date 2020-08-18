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
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace oslam
{
    DataReader::DataReader(const std::string &root_dir) : root_dir_(root_dir) { spdlog::debug("CONSTRUCT: DataReader"); }

    bool DataReader::run()
    {
        if (!shutdown_ && !dataset_parsed_)
        {
            dataset_parsed_ = parseDataset();
        }

        if (dataset_parsed_)
        {
            while (!shutdown_ && readFrame())
            {
                ++curr_idx_;
            }
        }

        if (shutdown_)
        {
            spdlog::info("DataReader Shutdown requested");
        }
        return false;
    }

    bool DataReader::parseDataset()
    {
        if (!fs::exists(root_dir_) || !fs::is_directory(root_dir_))
        {
            spdlog::error("Incorrect dataset path");
            return false;
        }

        fs::path color_files_path = root_dir_ / "color";
        fs::path depth_files_path = root_dir_ / "depth";
        fs::path mask_files_path  = root_dir_ / "preprocessed";
        fs::path intrinsic_path   = root_dir_ / "camera-intrinsics.json";

        if (!fs::exists(intrinsic_path) || !fs::is_regular_file(intrinsic_path))
        {
            spdlog::error("Could not find camera intrinsics");
            return false;
        }
        spdlog::debug("Color files path           : {}", color_files_path.string());
        spdlog::debug("Depth files path           : {}", depth_files_path.string());
        spdlog::debug("Groundtruth Mask files path: {}", mask_files_path.string());

        copy(fs::directory_iterator(color_files_path), fs::directory_iterator(), std::back_inserter(rgb_files_));
        copy(fs::directory_iterator(depth_files_path), fs::directory_iterator(), std::back_inserter(depth_files_));

        for (auto &file : fs::directory_iterator(mask_files_path))
        {
            if (file.path().extension() == ".png")
            {
                mask_files_.push_back(file);
            }
            else if (file.path().extension() == ".txt")
            {
                label_files_.push_back(file);
            }
            else
            {
                spdlog::error("Groundtruth segmentation has incorrect extension", file.path());
                return false;
            }
        }

        sort(rgb_files_.begin(), rgb_files_.end());
        sort(depth_files_.begin(), depth_files_.end());
        sort(mask_files_.begin(), mask_files_.end());

        if (rgb_files_.size() != depth_files_.size())
        {
            spdlog::error("Number of Color images and Depth images do not match");
            return false;
        }

        open3d::io::ReadIJsonConvertible(intrinsic_path.string(), intrinsic_);

        size_ = rgb_files_.size();
        spdlog::info("Total number of files: {}", rgb_files_.size());

        return true;
    }

    bool DataReader::readFrame()
    {
        if (curr_idx_ >= size_)
        {
            for (const auto &callback : shutdown_callbacks_)
            {
                callback(curr_idx_);
            }
            return false;
        }

        using namespace open3d::io;

        spdlog::debug("Reading files: \n{} \n{} \n{}",
                      rgb_files_.at(curr_idx_).string(),
                      depth_files_.at(curr_idx_).string(),
                      mask_files_.at(curr_idx_).string());

        cv::Mat color   = cv::imread(rgb_files_.at(curr_idx_).string(), cv::IMREAD_COLOR);
        cv::Mat depth   = cv::imread(depth_files_.at(curr_idx_).string(), cv::IMREAD_ANYDEPTH);

        // TODO:(Akash) Read ground truth pose from file conditionally based on input parameter
        /* cv::Mat gt_mask = cv::imread(mask_files_.at(curr_idx_).string()); */
        /* p_data->gt_pose = Eigen::Matrix4d::Identity(); */

        // Every 10th frame requires semantic segmentation
        for (const auto &callback : output_callbacks_)
        {
            callback(std::make_unique<Frame>(curr_idx_ + 1, color, depth, intrinsic_, (curr_idx_ % KEYFRAME_LENGTH) == 0));
        }
        return true;
    }

    void DataReader::shutdown()
    {
        spdlog::debug("Shutting down DataReader on demand");
        shutdown_ = true;
    }

}  // namespace oslam
