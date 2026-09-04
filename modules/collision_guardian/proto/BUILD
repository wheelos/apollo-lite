load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "collision_guardian_proto",
    srcs = ["collision_guardian.proto"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_proto",
    ],
)

cc_proto_library(
    name = "collision_guardian_cc_proto",
    deps = [":collision_guardian_proto"],
)

proto_library(
    name = "collision_guardian_config_proto",
    srcs = ["collision_guardian_config.proto"],
    deps = [":collision_guardian_proto"],
)

cc_proto_library(
    name = "collision_guardian_config_cc_proto",
    deps = [":collision_guardian_config_proto"],
)
