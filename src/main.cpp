/******************************************************************************
 * File:             main.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      main
 *****************************************************************************/

#ifndef NDEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>

#include "object-slam/utils/types.h"
#include "object-slam/controller/controller.h"

int main(int argc, char* argv[])
{
    CLI::App app{ "Object SLAM" };
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [func: %! line: %#] %^[%L]  %v%$");

    std::string dataset_path;
    app.add_option("dataset_path", dataset_path, "Path to the dataset")->required();

    int type;
    app.add_set("--dataset_type", type, {0, 1}, "Dataset type: 0 -> RGBD_SCENES, 1 -> TUM")->required();
    app.add_flag(
        "-d, --debug",
        [](size_t /*unused*/) {
#ifndef NDEBUG
            spdlog::set_level(spdlog::level::trace);
#else
            spdlog::set_level(spdlog::level::info);
#endif
        },
        "Print debug logs");

    spdlog::set_error_handler([](const std::string& msg) -> void {
        std::cerr << "Error in SPDLOG: " << msg << std::endl;
        raise(SIGABRT);
    });
    CLI11_PARSE(app, argc, argv);

    oslam::DatasetType dataset_type = static_cast<oslam::DatasetType>(type);
    spdlog::info("Dataset type: {} {}", type, dataset_type);

    oslam::Controller controller(dataset_path, dataset_type);

    if (controller.start())
    {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
