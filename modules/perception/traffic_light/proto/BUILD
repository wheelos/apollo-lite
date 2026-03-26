load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "traffic_light_component_cc_proto",
    deps = [":traffic_light_component_proto"],
)

proto_library(
    name = "traffic_light_component_proto",
    srcs = ["traffic_light_component.proto"],
)
