load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [
        ":routing_cast",
        ":routing_dump",
    ],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "routing/bin",
    strip_prefix = strip_prefix.from_root("modules/routing/tools"),
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
    name = "routing_dump",
    srcs = ["routing_dump.cc"],
    deps = [
        "//modules/routing:routing_service",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_binary(
    name = "routing_cast",
    srcs = ["routing_cast.cc"],
    deps = [
        "//modules/routing:routing_service",
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
