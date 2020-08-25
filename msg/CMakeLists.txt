find_package(Protobuf REQUIRED)

include(CMakePrintHelpers)

protobuf_generate_cpp(PROTO_SRCS PROTO_HEADERS image.proto)
protobuf_generate_python(PY_SRCS image.proto)

add_library(proto ${PROTO_SRCS} ${PROTO_HEADERS})
target_include_directories(
    proto PRIVATE
  ${Protobuf_INCLUDE_DIR}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(proto PUBLIC ${Protobuf_LIBRARY})

add_custom_target(proto_py ALL DEPENDS ${PY_SRCS})
