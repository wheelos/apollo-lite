## Auto generated by `proto_build_generator.sh`
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "system_status_cc_proto",
    deps = [
        ":system_status_proto",
    ],
)

proto_library(
    name = "system_status_proto",
    srcs = ["system_status.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "system_status_py_pb2",
    deps = [":system_status_proto"],
)

cc_proto_library(
    name = "smart_recorder_status_cc_proto",
    deps = [
        ":smart_recorder_status_proto",
    ],
)

proto_library(
    name = "smart_recorder_status_proto",
    srcs = ["smart_recorder_status.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "smart_recorder_status_py_pb2",
    deps = [":smart_recorder_status_proto"],
)

cc_proto_library(
    name = "monitor_log_cc_proto",
    deps = [
        ":monitor_log_proto",
    ],
)

proto_library(
    name = "monitor_log_proto",
    srcs = ["monitor_log.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "monitor_log_py_pb2",
    deps = [":monitor_log_proto"],
)
