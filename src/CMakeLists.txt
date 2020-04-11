find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(reconstruct main.cpp dataset.cpp frame.cpp image_transport.cpp)
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
