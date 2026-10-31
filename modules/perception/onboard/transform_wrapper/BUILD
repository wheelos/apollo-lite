load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

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
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:ground_reference",
        "@wheelos_transform//modules/transform:timed_transform_resolver",
        "@wheelos_transform//modules/transform:transform_query",
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
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:buffer_test_peer",
        "@wheelos_transform//modules/transform:static_transform_loader",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

pkg_files(
    name = "package_files_archives",
    srcs = [
        ":sensor_calibration",
        ":transform_wrapper",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/perception/onboard/transform_wrapper"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
