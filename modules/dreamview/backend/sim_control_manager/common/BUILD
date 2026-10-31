load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_archives",
    srcs = [":sim_control_gflags"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend/sim_control_manager/common"),
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

cc_library(
    name = "sim_control_gflags",
    srcs = [
        "sim_control_gflags.cc",
    ],
    hdrs = ["sim_control_gflags.h"],
    deps = [
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = [
        "interpolation_2d.cc",
    ],
    hdrs = [
        "interpolation_2d.h",
    ],
)

cc_library(
    name = "sim_control_util",
    srcs = [
        "sim_control_util.cc",
    ],
    hdrs = [
        "sim_control_util.h",
    ],
    deps = [
        "@core//cyber",
    ],
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
