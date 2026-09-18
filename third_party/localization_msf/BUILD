load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

install(
    name = "install",
    data = [
        "cyberfile.xml",
    ],
    data_dest = "3rd-localization-msf",
)

install_src_files(
    name = "install_src",
    dest = "3rd-localization-msf/src",
    filter = "*",
    src_dir = ["."],
    deps = [":install_src_lib"],
)

install_src_files(
    name = "install_src_lib",
    dest = "3rd-localization-msf/lib",
    filter = "*",
    src_dir = if_x86_64([
        "x86_64/lib",
    ]) + if_aarch64([
        "aarch64/lib",
    ]),
)
