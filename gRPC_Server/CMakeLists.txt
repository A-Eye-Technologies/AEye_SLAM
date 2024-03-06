cmake_minimum_required(VERSION 3.8)

project(VSLAM C CXX)

include(common.cmake)

find_package(OpenCV REQUIRED)

# Proto file
get_filename_component(vslam_proto "protos/vslam.proto" ABSOLUTE)
get_filename_component(vslam_proto_path "${vslam_proto}" PATH)

# Generated sources
set(vslam_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/vslam.pb.cc")
set(vslam_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/vslam.pb.h")
set(vslam_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/vslam.grpc.pb.cc")
set(vslam_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/vslam.grpc.pb.h")
add_custom_command(
      OUTPUT "${vslam_proto_srcs}" "${vslam_proto_hdrs}" "${vslam_grpc_srcs}" "${vslam_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${vslam_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${vslam_proto}"
      DEPENDS "${vslam_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# vslam_grpc_proto
add_library(vslam_grpc_proto
  ${vslam_grpc_srcs}
  ${vslam_grpc_hdrs}
  ${vslam_proto_srcs}
  ${vslam_proto_hdrs})
target_link_libraries(vslam_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

# Targets greeter_[async_](client|server)
foreach(_target vslam_server)
  add_executable(${_target} "${_target}.cc")
  target_link_libraries(${_target}
    vslam_grpc_proto
    absl::flags
    absl::flags_parse
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
    ${OpenCV_LIBS}) 
endforeach()
