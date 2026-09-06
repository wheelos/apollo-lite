load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "localization_health_proto",
    srcs = ["localization_health.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "localization_health_cc_proto",
    deps = [
        ":localization_health_proto",
    ],
)

py_proto_library(
    name = "localization_health_py_pb2",
    deps = [
        ":localization_health_proto",
    ],
)
