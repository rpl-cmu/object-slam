/******************************************************************************
 * File:             main.cpp
 *
 * Author:           Akash Sharma
 * Created:          04/07/20
 * Description:      main
 *****************************************************************************/
#include <docopt/docopt.h>
#include "controller.h"

static constexpr auto USAGE =
  R"(Object SLAM Pipeline

    Usage:
          reconstruct [options] <dataset_path>

    Options:
          -h --help    Show the help screen
          -v --vis     Visualize the output
          -d --debug   Print debug logs
    )";

int main(int argc, char *argv[])
{


    std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
      { std::next(argv), std::next(argv, argc) },
      true,// show help if requested
      "Object SLAM 0.1");// version string

    oslam::Controller controller(args);

    controller.run();
}
