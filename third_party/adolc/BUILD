load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":3rd-adolc.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-adolc",
)

install_src_files(
    name = "install_src",
    dest = "3rd-adolc/src",
    filter = "*",
    src_dir = ["."],
)
