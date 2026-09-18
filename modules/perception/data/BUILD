load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "perception/data",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "params/*.yaml",
    ]),
)
