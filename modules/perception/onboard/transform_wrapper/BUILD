load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transform_wrapper",
    srcs = [
        "transform_wrapper.cc",
    ],
    hdrs = ["transform_wrapper.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "//modules/transform:ground_reference",
        "//modules/transform:timed_transform_resolver",
        "//modules/transform:transform_query",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
        "@wheelos_common//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "sensor_calibration",
    srcs = ["sensor_calibration.cc"],
    hdrs = ["sensor_calibration.h"],
    deps = [
        ":transform_wrapper",
        "//modules/perception/common/sensor_manager",
        "@core//cyber/common:log",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:lidar_height",
    ],
)

cc_test(
    name = "sensor_calibration_test",
    srcs = ["sensor_calibration_test.cc"],
    deps = [
        ":sensor_calibration",
        ":transform_wrapper",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "//modules/transform:buffer_test_peer",
        "//modules/transform:static_transform_loader",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":transform_wrapper",
        ":sensor_calibration",
    ],
    visibility = ["//visibility:public"],
)

cpplint()
