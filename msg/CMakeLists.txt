find_package(Protobuf REQUIRED)

include(CMakePrintHelpers)

protobuf_generate_cpp(PROTO_SRCS PROTO_HEADERS image.proto)
protobuf_generate_python(PY_SRCS image.proto)

add_library(proto ${PROTO_SRCS} ${PROTO_HEADERS})
target_include_directories(proto PRIVATE ${Protobuf_INCLUDE_DIR})

add_custom_target(proto_py ALL DEPENDS ${PY_SRCS})
