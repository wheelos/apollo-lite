load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "*.bin",
    ]) + ["SHA256SUMS"],
)

install(
    name = "install",
    data = [":runtime_data"],
    data_dest = "assets/sites/map/borregas_ave",
)
