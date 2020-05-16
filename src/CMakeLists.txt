find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(
  reconstruct
  main.cpp
  frame.cpp
  masked_image.cpp
  controller.cpp
  data_reader.cpp
  data_provider.cpp
  image_transport.cpp
  tracker.cpp
  # tsdf_object.cpp
  # map.cpp
)
add_subdirectory(utils)

target_link_libraries(
  reconstruct PRIVATE project_options project_warnings
  CONAN_PKG::docopt.cpp
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cppzmq
  CONAN_PKG::protobuf
  Boost::filesystem
  Eigen3::Eigen
  Open3D::Open3D
  opencv_core
  opencv_highgui
  ${CMAKE_DL_LIBS}
  proto
)

target_include_directories(reconstruct PRIVATE ${CMAKE_BINARY_DIR})

if(WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(reconstruct PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()
