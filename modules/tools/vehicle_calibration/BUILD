load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "validate_calibration",
    srcs = ["validate_calibration.cc"],
    deps = [
        "//modules/perception/common/sensor_manager",
        "@wheelos_transform//modules/transform:static_transform_loader",
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

pkg_files(
    name = "package_files_binaries",
    srcs = [":validate_calibration"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "tools/vehicle_calibration",
    strip_prefix = strip_prefix.from_root("modules/tools/vehicle_calibration"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
