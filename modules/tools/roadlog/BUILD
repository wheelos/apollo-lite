load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":smart_recorder"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "tools/roadlog",
    strip_prefix = strip_prefix.from_root("modules/tools/roadlog"),
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

cc_binary(
    name = "smart_recorder",
    srcs = ["smart_recorder.cc"],
    linkstatic = True,
    deps = [
        ":smart_recorder_gflags",
        "//modules/tools/roadlog/processor:roadlog_runtime",
    ],
)

cc_library(
    name = "smart_recorder_gflags",
    srcs = ["smart_recorder_gflags.cc"],
    hdrs = ["smart_recorder_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)
