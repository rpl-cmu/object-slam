include(${PROJECT_SOURCE_DIR}/cmake/Functions.cmake)
# --------- Dependencies -----------------------------------------
find_package(GTSAM REQUIRED)

# include Open3D library by downloading if not already present
# include(${PROJECT_SOURCE_DIR}/cmake/Open3D.cmake)
find_package(Open3D REQUIRED
             COMPONENTS Open3DCuda)

# --------- Create object-slam library ----------------------------
add_library(${PROJECT_NAME})

add_subdirectory(utils)
add_subdirectory(struct)
add_subdirectory(controller)
add_subdirectory(reader)
add_subdirectory(module)

# Treat Eigen as system library to suppress warnings
target_include_directories(
  ${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/src/
  SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_BINARY_DIR}
)

target_link_libraries_system(
  ${PROJECT_NAME} PUBLIC
  project_options
  project_warnings
  proto
  CONAN_PKG::CLI11
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cppzmq
  CONAN_PKG::protobuf
  Threads::Threads
  Boost::filesystem
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_rgbd
  Open3D::Open3D
  Open3D::Open3DCuda
  gtsam
)

# TODO: Is this Required?
if(WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(
      ${PROJECT_NAME} PUBLIC
      OpenMP::OpenMP_CXX
    )
  endif()
endif()
