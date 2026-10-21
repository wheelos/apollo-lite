load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "validate_calibration",
    srcs = ["validate_calibration.cc"],
    deps = [
        "//modules/perception/common/sensor_manager",
        "//modules/transform:static_transform_loader",
        "@wheelos_common//modules/common/vehicle_calibration:lidar_height",
    ],
)

cc_test(
    name = "validate_calibration_test",
    size = "small",
    srcs = ["validate_calibration_test.cc"],
    data = [":validate_calibration"],
    deps = [
        "@bazel_tools//tools/cpp/runfiles",
        "@com_google_googletest//:gtest_main",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

install(
    name = "install",
    runtime_dest = "tools/vehicle_calibration",
    targets = [":validate_calibration"],
)

cpplint()
