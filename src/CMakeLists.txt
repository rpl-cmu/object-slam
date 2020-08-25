add_subdirectory(object-slam)

add_executable(
  reconstruct
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_link_libraries(reconstruct
    PRIVATE
    ${PROJECT_NAME}
    )
