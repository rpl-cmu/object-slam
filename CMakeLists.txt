cmake_minimum_required(VERSION 3.15)

project(object-slam LANGUAGES CXX CUDA)
include(cmake/StandardProjectSettings.cmake)

# ---------- Standard Project settings and warnings -----------------
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# enable doxygen
include(cmake/Doxygen.cmake)
enable_doxygen()

# ---------- Options for debugging -----------

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

option(WITH_DEBUG_VIS "With Visualization for debugging" ON)

if(WITH_DEBUG_VIS)
  target_compile_definitions(project_options INTERFACE OSLAM_DEBUG_VIS)
endif()

# ---------- Add Dependencies -----------
find_package(Threads REQUIRED)

find_package(Boost REQUIRED)

# Set up some extra Conan dependencies based on our needs before loading Conan
set(CONAN_EXTRA_REQUIRES "")
set(CONAN_EXTRA_OPTIONS "")
include(cmake/Conan.cmake)
run_conan()

find_package(Eigen3 3.3 REQUIRED)

find_package(OpenCV 4.0 REQUIRED
             COMPONENTS core imgproc highgui rgbd)


# ------------------ Install configuration------------------
# TODO
# ------------------ Build options --------------

option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" OFF)
option(ENABLE_PCH "Enable Precompiled Headers" OFF)
option(WITH_OPENMP "Use OpenMP" ON)
if(ENABLE_PCH)
  target_precompile_headers(project_options INTERFACE <vector> <string> <map> <utility>)
endif()

add_subdirectory(msg)
add_subdirectory(src)

if(ENABLE_TESTING)
  enable_testing()
  message("Building Tests. Be sure to check out test/constexpr_tests for constexpr testing")
  add_subdirectory(test)
endif()
