load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data = [
#         ":runtime_data",
#     ],
#     data_dest = "drivers/addition_data/lidar/lslidar",
#     library_dest = "drivers/lib/lidar/lslidar",
#     targets = [
#         "//modules/drivers/lidar/lslidar/driver:liblslidar_driver_component.so",
#         "//modules/drivers/lidar/lslidar/parser:liblslidar_convert_component.so",
#     ],
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/*.yaml",
    ]),
)

cc_binary(
    name = "liblslidar_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":lslidar_component_lib",
    ],
)

cc_library(
    name = "lslidar_component_lib",
    srcs = ["component/lslidar_component.cpp"],
    hdrs = ["component/lslidar_component.h"],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/lslidar/driver",
        "//modules/drivers/lidar/lslidar/parser:convert",
    ],
    alwayslink = True,
)

cpplint()
