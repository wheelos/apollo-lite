load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "lane_component_config_proto",
    srcs = ["lane_component_config.proto"],
)

cc_proto_library(
    name = "lane_component_config_cc_proto",
    deps = [":lane_component_config_proto"],
)
