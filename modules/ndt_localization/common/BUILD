load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_gflags",
    srcs = ["localization_gflags.cc"],
    hdrs = ["localization_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "rigid_transform_helper",
    srcs = ["rigid_transform_helper.cc"],
    hdrs = ["rigid_transform_helper.h"],
    deps = [
        "@wheelos_transform//modules/transform:transform_query",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

pkg_files(
    name = "package_files_archives",
    srcs = [
        ":localization_gflags",
        ":rigid_transform_helper",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/ndt_localization/common"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
