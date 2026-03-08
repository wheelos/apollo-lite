load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":3rd-can-card-library.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-can-card-library",
)

install_src_files(
    name = "install_src",
    deps = [
        ":src",
    ],
)

install_src_files(
    name = "src",
    dest = "3rd-can-card-library/src",
    filter = "*",
    src_dir = ["."],
)
