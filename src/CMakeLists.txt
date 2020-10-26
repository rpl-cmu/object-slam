add_subdirectory(object-slam)

add_executable(
  reconstruct
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_executable(
  show_objects
  ${CMAKE_CURRENT_SOURCE_DIR}/show-objects.cpp
)

target_link_libraries(
  reconstruct
  PRIVATE
  ${PROJECT_NAME}
)

target_link_libraries(
  show_objects
  PRIVATE
  ${PROJECT_NAME}
)
