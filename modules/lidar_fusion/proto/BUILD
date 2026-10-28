load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "lidar_fusion_proto",
    srcs = ["lidar_fusion.proto"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "lidar_fusion_cc_proto",
    deps = [":lidar_fusion_proto"],
)
