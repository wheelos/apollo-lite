load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "common",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    targets = [
        ":vehicle_state_provider",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vehicle_state_provider",
    srcs = [
        "vehicle_state_provider.cc",
    ],
    hdrs = ["vehicle_state_provider.h"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_test(
    name = "vehicle_state_provider_test",
    size = "small",
    srcs = ["vehicle_state_provider_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":vehicle_state_provider",
        "//modules/common/configs:config_gflags",
        "//modules/planning/common:gtest_exit_main",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cpplint()
