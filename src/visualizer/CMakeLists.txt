# -------------------- Dependencies -----------------------

find_package(Pangolin REQUIRED)

add_library(
  Visualizer
  visualizer.h
  visualizer.cpp
)

target_include_directories(
  viewer
  PUBLIC
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  ${Pangolin_INCLUDE_DIR}
)

target_link_libraries(
  viewer
  PUBLIC
  ${PROJECT_NAME}
  pangolin
)
