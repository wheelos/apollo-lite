load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

COLLISION_GUARDIAN_COPTS = ['-DMODULE_NAME=\\"collision_guardian\\"']

cc_library(
    name = "risk_filter",
    srcs = ["core/risk_filter.cc"],
    hdrs = ["core/risk_filter.h"],
)

cc_library(
    name = "occupancy_detector",
    srcs = ["core/occupancy_detector.cc"],
    hdrs = ["core/occupancy_detector.h"],
    deps = ["@eigen"],
)

cc_library(
    name = "collision_guardian_component_lib",
    srcs = ["collision_guardian_component.cc"],
    hdrs = ["collision_guardian_component.h"],
    copts = COLLISION_GUARDIAN_COPTS,
    deps = [
        ":occupancy_detector",
        ":risk_filter",
        "//modules/collision_guardian/proto:collision_guardian_cc_proto",
        "//modules/collision_guardian/proto:collision_guardian_config_cc_proto",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:transform_query",
        "@core//cyber",
        "@eigen",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcollision_guardian_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":collision_guardian_component_lib"],
)

cc_test(
    name = "risk_filter_test",
    size = "small",
    srcs = ["core/risk_filter_test.cc"],
    deps = [
        ":risk_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "occupancy_detector_test",
    size = "small",
    srcs = ["core/occupancy_detector_test.cc"],
    deps = [
        ":occupancy_detector",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ]),
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "collision_guardian",
    strip_prefix = strip_prefix.from_root("modules/collision_guardian"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libcollision_guardian_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "collision_guardian/lib",
    strip_prefix = strip_prefix.from_root("modules/collision_guardian"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
