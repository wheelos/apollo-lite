load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":vehicle_reference",
        ":vehicle_state_provider",
    ],
    visibility = ["//visibility:public"],
)

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "vehicle_reference",
    srcs = [
        "reference_point_transformer.cc",
        "vehicle_description.cc",
        "vehicle_geometry_model.cc",
    ],
    hdrs = [
        "reference_point_transformer.h",
        "vehicle_description.h",
        "vehicle_geometry_model.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
    ],
)

cc_library(
    name = "vehicle_state_provider",
    srcs = ["vehicle_state_provider.cc"],
    hdrs = ["vehicle_state_provider.h"],
    deps = [
        "//modules/common/configs:config_gflags",
        ":vehicle_reference",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "vehicle_state_provider_test",
    size = "small",
    srcs = ["vehicle_state_provider_test.cc"],
    deps = [
        ":vehicle_state_provider",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/planning/common:gtest_exit_main",
    ],
)

cc_test(
    name = "reference_point_transformer_test",
    size = "small",
    srcs = ["reference_point_transformer_test.cc"],
    deps = [
        ":vehicle_reference",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
    ],
)

cc_test(
    name = "vehicle_geometry_model_test",
    size = "small",
    srcs = ["vehicle_geometry_model_test.cc"],
    deps = [
        ":vehicle_reference",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
