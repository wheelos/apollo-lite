load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":3rd-opengl.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-opengl",
)

install_src_files(
    name = "install_src",
    dest = "3rd-opengl/src",
    filter = "*",
    src_dir = ["."],
)
