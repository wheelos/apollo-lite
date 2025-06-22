load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":vehicle_state_provider",
    ],
    visibility = ["//visibility:public"],
)

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "vehicle_state_provider",
    srcs = [
        "vehicle_state_provider.cc",
    ],
    hdrs = ["vehicle_state_provider.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@eigen",
        "@com_google_absl//absl/strings",
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
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "@eigen",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
