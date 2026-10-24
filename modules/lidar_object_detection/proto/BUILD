load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "pointpillars_proto",
    srcs = ["pointpillars.proto"],
    deps = ["@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto"],
)

cc_proto_library(
    name = "pointpillars_cc_proto",
    deps = [":pointpillars_proto"],
)

proto_library(
    name = "lidar_object_detection_proto",
    srcs = ["lidar_object_detection.proto"],
    deps = ["@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto"],
)

cc_proto_library(
    name = "lidar_object_detection_cc_proto",
    deps = [":lidar_object_detection_proto"],
)
