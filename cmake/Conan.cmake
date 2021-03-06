macro(run_conan)
  # Download automatically, you can also just copy the conan.cmake file
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(
      STATUS
      "Downloading conan.cmake from https://github.com/conan-io/cmake-conan"
    )
    file(
      DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake"
      "${CMAKE_BINARY_DIR}/conan.cmake"
    )
  endif()

  include(${CMAKE_BINARY_DIR}/conan.cmake)

  conan_add_remote(
    NAME bincrafters URL
    https://api.bintray.com/conan/bincrafters/public-conan
  )

  conan_add_remote(
    NAME omaralvarez URL
    https://api.bintray.com/conan/omaralvarez/public-conan
  )

  conan_cmake_run(
    REQUIRES
    ${CONAN_EXTRA_REQUIRES}
    catch2/2.11.0
    CLI11/1.9.1@cliutils/stable
    fmt/7.0.3
    spdlog/1.8.0
    cppzmq/4.6.0
    protoc_installer/3.9.1@bincrafters/stable
    protobuf/3.9.1@bincrafters/stable
    xtensor/0.20.10@omaralvarez/public-conan
    OPTIONS
    ${CONAN_EXTRA_OPTIONS}
    BASIC_SETUP
    CMAKE_TARGETS # individual targets to link to
    BUILD
    missing
  )
endmacro()
