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
#include <opencv2/rgbd/depth.hpp>
#include <vector>

namespace oslam
{
    DataReader::DataReader(std::string root_dir, DatasetType dataset_type)
        : dataset_type_(dataset_type), root_dir_(std::move(root_dir))
    {
        spdlog::trace("CONSTRUCT: DataReader");
    }

    bool DataReader::run()
    {
        spdlog::trace("DataReader::run()");
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
        spdlog::trace("DataReader::parseDataset()");
        if (!fs::exists(root_dir_) || !fs::is_directory(root_dir_))
        {
            spdlog::error("Incorrect dataset path");
            return false;
        }

        if (dataset_type_ == DatasetType::RGBD_SCENES)
        {
            return parseRgbdScenes();
        }
        else
        {
            return parseTum();
        }
        return false;
    }

    bool DataReader::parseRgbdScenes()
    {
        spdlog::trace("DataReader::parseRgbdScenes()");
        bool mask_files_exist = true;

        fs::path color_files_path = root_dir_ / "color";
        fs::path depth_files_path = root_dir_ / "depth";
        fs::path mask_files_path  = root_dir_ / "preprocessed";
        fs::path intrinsic_path   = root_dir_ / "camera-intrinsics.json";

        if (!fs::exists(intrinsic_path) || !fs::is_regular_file(intrinsic_path))
        {
            spdlog::error("Could not find camera intrinsics");
            return false;
        }

        spdlog::info("Found camera intrinsics");
        if (!fs::exists(color_files_path) || !fs::exists(depth_files_path) || !fs::is_directory(color_files_path) ||
            !fs::is_directory(depth_files_path))
        {
            spdlog::error("Could not find color and depth images at path \n or path {}\n {}\n is not a directory",
                          color_files_path.string(),
                          depth_files_path.string());
            return false;
        }
        spdlog::info("Found files in the color and depth folder");

        if (!fs::exists(mask_files_path) || !fs::is_directory(mask_files_path))
        {
            spdlog::info("Could not find mask files at path:", mask_files_path.string());
            mask_files_exist = false;
        }

        spdlog::info("Found Color files path : {}", color_files_path.string());
        spdlog::info("Found Depth files path : {}", depth_files_path.string());

        copy(fs::directory_iterator(color_files_path), fs::directory_iterator(), std::back_inserter(rgb_files_));
        copy(fs::directory_iterator(depth_files_path), fs::directory_iterator(), std::back_inserter(depth_files_));
        sort(rgb_files_.begin(), rgb_files_.end());
        sort(depth_files_.begin(), depth_files_.end());

        if (mask_files_exist)
        {
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
                    spdlog::error("Groundtruth segmentation has incorrect extension", file.path().string());
                    return false;
                }
            }
            sort(mask_files_.begin(), mask_files_.end());
        }

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

    bool DataReader::parseTum()
    {
        spdlog::trace("DataReader::parseTum()");
        fs::path color_files_path      = root_dir_ / "rgb";
        fs::path depth_files_path      = root_dir_ / "depth";
        fs::path associated_files_path = root_dir_ / "files.txt";
        fs::path intrinsic_path        = root_dir_ / "camera-intrinsics.json";

        if (!fs::exists(associated_files_path) || !fs::is_regular_file(associated_files_path))
        {
            spdlog::error("Please generate associated files, since data is unsynchronized");
            return false;
        }

        if (!fs::exists(intrinsic_path) || !fs::is_regular_file(intrinsic_path))
        {
            spdlog::error("Could not find camera intrinsics");
            return false;
        }

        spdlog::info("Found camera intrinsics");
        if (!fs::exists(color_files_path) || !fs::exists(depth_files_path) || !fs::is_directory(color_files_path) ||
            !fs::is_directory(depth_files_path))
        {
            spdlog::error("Could not find color and depth images at path \n or path {}\n {}\n is not a directory",
                          color_files_path.string(),
                          depth_files_path.string());
            return false;
        }

        spdlog::info("Found files in the color and depth folder");

        spdlog::info("Found Color files path : {}", color_files_path.string());
        spdlog::info("Found Depth files path : {}", depth_files_path.string());

        fs::ifstream associated_files{ associated_files_path };

        spdlog::info("Opened file: {}", associated_files_path.string());

        std::string s;
        std::string delimiter{ " " };
        while (std::getline(associated_files, s))
        {
            size_t pos_start = 0, pos_end, delim_len = delimiter.length();
            std::string token;
            std::vector<std::string> tokens;

            while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
            {
                token     = s.substr(pos_start, pos_end - pos_start);
                pos_start = pos_end + delim_len;
                tokens.push_back(token);
            }
            tokens.push_back(s.substr(pos_start));
            if (tokens.size() != 4)
            {
                spdlog::error("Check file {} whether it has 4 columns of data", associated_files_path.string());
                return false;
            }
            const std::string &rgb_filename   = tokens.at(1);
            const std::string &depth_filename = tokens.at(3);

            fs::path rgb_file   = root_dir_ / rgb_filename;
            fs::path depth_file = root_dir_ / depth_filename;

            if (!fs::exists(rgb_file) || !fs::is_regular_file(rgb_file))
            {
                spdlog::error("Could not find {}", (rgb_file).string());
                return false;
            }
            if (!fs::exists(depth_file) || !fs::is_regular_file(depth_file))
            {
                spdlog::error("Could not find {}", (depth_file).string());
                return false;
            }
            rgb_files_.push_back(rgb_file);
            depth_files_.push_back(depth_file);
        }

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
        spdlog::trace("DataReader::readFrame()");
        if (curr_idx_ >= size_)
        {
            for (const auto &callback : shutdown_callbacks_)
            {
                callback(curr_idx_);
            }
            return false;
        }

        using namespace open3d::io;

        spdlog::debug("Reading files: \n{} \n{}", rgb_files_.at(curr_idx_).string(), depth_files_.at(curr_idx_).string());

        cv::Mat color      = cv::imread(rgb_files_.at(curr_idx_).string(), cv::IMREAD_COLOR);
        cv::Mat depth      = cv::imread(depth_files_.at(curr_idx_).string(), cv::IMREAD_ANYDEPTH);
        float depth_factor = 1000.0f;
        float max_depth    = 3.0f;
        if (dataset_type_ == DatasetType::TUM)
        {
            depth_factor = 5000.0f;
            spdlog::trace("Setting depth factor = 5000.0");
        }

        // Every 10th frame requires semantic segmentation
        for (const auto &callback : output_callbacks_)
        {
            callback(std::make_unique<Frame>(
                curr_idx_ + 1, color, depth, intrinsic_, (curr_idx_ % KEYFRAME_LENGTH) == 0, depth_factor, max_depth));
        }
        return true;
    }

    void DataReader::shutdown()
    {
        spdlog::info("Shutting down DataReader on demand");
        shutdown_ = true;
    }

}  // namespace oslam
