find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED COMPONENTS Open3DCuda)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui rgbd)

add_executable(
  reconstruct
  main.cpp
  controller.cpp
  data_reader.cpp
  data_provider.cpp
  image_transport.cpp
  tsdf_object.cpp
  map.cpp
  tracker.cpp
  mapper.cpp
  renderer.cpp
)

add_subdirectory(utils)

# Special handling to suppress warnings from Eigen headers
target_include_directories(
  reconstruct SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_BINARY_DIR}
)

target_link_libraries(
  reconstruct PRIVATE project_options project_warnings
  CONAN_PKG::docopt.cpp
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cppzmq
  CONAN_PKG::protobuf
  Boost::filesystem
  Open3D::Open3D
  Open3D::Open3DCuda
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_rgbd
  ${CMAKE_DL_LIBS}
  proto
)

#
if(WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(reconstruct PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()
