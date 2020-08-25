/******************************************************************************
 * File:             main.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      main
 *****************************************************************************/
#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>

#include "object-slam/controller/controller.h"

int main(int argc, char *argv[])
{
    CLI::App app{ "Object SLAM" };
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");

    std::string dataset_path;
    app.add_option("dataset_path", dataset_path, "Path to the dataset")->required();

    app.add_flag(
        "-d, --debug", [](size_t /*unused*/) { spdlog::set_level(spdlog::level::debug); }, "Print debug logs");

    CLI11_PARSE(app, argc, argv);

    oslam::Controller controller(dataset_path);

    if (controller.start())
    {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
