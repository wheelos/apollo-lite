load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "environment_model",
    hdrs = ["environment_model.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_library(
    name = "environment_model_builder",
    srcs = ["environment_model_builder.cc"],
    hdrs = ["environment_model_builder.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":environment_model",
        "//modules/planning/common:local_view",
    ],
)

cc_library(
    name = "capability_extractor",
    srcs = ["capability_extractor.cc"],
    hdrs = ["capability_extractor.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":environment_model",
    ],
)

cpplint()
