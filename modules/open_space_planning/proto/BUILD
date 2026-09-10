load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "open_space_planning_proto",
    srcs = ["open_space_planning.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "open_space_planning_cc_proto",
    deps = [":open_space_planning_proto"],
)
