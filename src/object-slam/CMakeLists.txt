# --------- Dependencies ------------
find_package(GTSAM REQUIRED)

find_package(
  Open3D REQUIRED
  COMPONENTS Open3DCuda
)

add_executable(
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_subdirectory(utils)
add_subdirectory(struct)
add_subdirectory(controller)
add_subdirectory(reader)
add_subdirectory(module)

# Special handling to suppress warnings from Eigen headers
target_include_directories(
  ${PROJECT_NAME} SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_BINARY_DIR}
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}/src/
)

target_link_libraries(
  ${PROJECT_NAME} PRIVATE
  project_options
  project_warnings
  CONAN_PKG::CLI11
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cppzmq
  CONAN_PKG::protobuf
  Threads::Threads
  Boost::filesystem
  proto
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_rgbd
  Open3D::Open3D
  Open3D::Open3DCuda
  gtsam
)

#
if(WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(
      ${PROJECT_NAME} PRIVATE
      OpenMP::OpenMP_CXX
    )
  endif()
endif()
