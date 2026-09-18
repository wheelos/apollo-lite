load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "localization/addition_data/msf/params",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "gnss_params/*.yaml",
        "vehicle_params/*.yaml",
        "velodyne_params/*.yaml",
        "*.yaml",
    ]),
)
