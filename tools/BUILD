load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(
    default_visibility = ["//visibility:public"],
)

alias(
    name = "mainboard",
    actual = "@core//tools:mainboard",
)

alias(
    name = "cyber_launch",
    actual = "@core//tools:cyber_launch",
)

alias(
    name = "cyber_recorder",
    actual = "@core//tools:cyber_recorder",
)

alias(
    name = "cyber_monitor",
    actual = "@core//tools:cyber_monitor",
)

alias(
    name = "cyber_channel",
    actual = "@core//tools:cyber_channel",
)

alias(
    name = "cyber_node",
    actual = "@core//tools:cyber_node",
)

alias(
    name = "cyber_service",
    actual = "@core//tools:cyber_service",
)

exports_files(["CPPLINT.cfg"])

pkg_files(
    name = "source_files_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//tools/hardware:exported_source_files",
        "//tools/modules:exported_source_files",
        "//tools/platform:exported_source_files",
        "//tools/transform:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "bazel-extend-tools/src",
    strip_prefix = strip_prefix.from_root("tools"),
)

pkg_filegroup(
    name = "source_files",
    srcs = [":source_files_data"],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)
