load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])
PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

install(
    name = "install",
    library_dest = "prediction/lib",
    targets = [
        ":evaluator_submodule_lib",
        ":predictor_submodule_lib",
        ":prediction_lego_lib",
        ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "submodule_output",
    srcs = ["submodule_output.cc"],
    hdrs = ["submodule_output.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cc_library(
    name = "evaluator_submodule_lib",
    srcs = ["evaluator_submodule.cc"],
    hdrs = ["evaluator_submodule.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
    ],
    alwayslink = True,
)

cc_library(
    name = "predictor_submodule_lib",
    srcs = ["predictor_submodule.cc"],
    hdrs = ["predictor_submodule.h"],
    copts = PREDICTION_COPTS,
    deps = [
        ":evaluator_submodule_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "prediction_lego_lib",
    deps = [
        ":evaluator_submodule_lib",
        ":predictor_submodule_lib",
        "//modules/prediction:prediction_component_lib",
    ],
)

cpplint()
