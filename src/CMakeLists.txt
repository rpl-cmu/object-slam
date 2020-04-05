find_package(GTSAM REQUIRED)
find_package(Open3D REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(reconstruct main.cpp dataset.cpp)
target_link_libraries(
  reconstruct PRIVATE project_options project_warnings CONAN_PKG::docopt.cpp
  CONAN_PKG::fmt CONAN_PKG::spdlog Boost::filesystem Eigen3::Eigen Open3D::Open3D ${CMAKE_DL_LIBS}
)

if(WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(reconstruct PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()
