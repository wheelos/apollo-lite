load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_flags",
    srcs = ["common_flags.cc"],
    hdrs = ["common_flags.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

pkg_files(
    name = "package_files_archives",
    srcs = [":common_flags"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/perception/onboard/common_flags"),
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
