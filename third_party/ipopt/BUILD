load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":3rd-ipopt.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-ipopt",
)

install_src_files(
    name = "install_src",
    dest = "3rd-ipopt/src",
    filter = "*",
    src_dir = ["."],
)
