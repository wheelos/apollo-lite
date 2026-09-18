load("//tools/install:install.bzl", "install", "install_src_files")

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

install(
    name = "install",
    data = [
        ":cyberfile.xml",
    ],
    data_dest = "bazel-extend-tools",
)

install_src_files(
    name = "install_src",
    dest = "bazel-extend-tools/src",
    filter = "*",
    src_dir = ["."],
)
