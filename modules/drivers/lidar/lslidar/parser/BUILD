load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "convert",
    srcs = [
        "calibration.cc",
        "convert.cc",
        "lslidar16_parser.cc",
        "lslidar32_parser.cc",
        "lslidarCH120_parser.cc",
        "lslidarCH128X1_parser.cc",
        "lslidarCH128_parser.cc",
        "lslidarCH16_parser.cc",
        "lslidarCH32_parser.cc",
        "lslidarCH64_parser.cc",
        "lslidarCH64w_parser.cc",
        "lslidarCXV4_parser.cc",
        "lslidarLS128S2_parser.cc",
        "lslidar_parser.cc",
    ],
    hdrs = [
        "calibration.h",
        "convert.h",
        "lslidar_parser.h",
    ],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/lslidar/proto:config_cc_proto",
        "//modules/drivers/lidar/lslidar/proto:lslidar_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cpplint()
