load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_lidar_locator",
    srcs = ["lidar_locator_ndt.cc"],
    hdrs = [
        "lidar_locator_ndt.h",
        "ndt_solver.h",
        "ndt_solver.hpp",
        "ndt_voxel_grid_covariance.h",
        "ndt_voxel_grid_covariance.hpp",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/ndt_map",
        "//modules/localization/msf/local_pyramid_map/ndt_map:ndt_map_pool",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@eigen",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:registration",
    ],
)

cc_test(
    name = "ndt_lidar_locator_test",
    size = "large",
    timeout = "short",
    srcs = ["lidar_locator_ndt_test.cc"],
    data = ["//modules/ndt_localization:test_data"],
    deps = [
        ":ndt_lidar_locator",
        "//modules/localization/msf/common/io:common_io",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cc_test(
    name = "ndt_solver_test",
    size = "large",
    timeout = "short",
    srcs = ["ndt_solver_test.cc"],
    data = ["//modules/ndt_localization:test_data"],
    deps = [
        ":ndt_lidar_locator",
        "//modules/localization/msf/common/io:common_io",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cpplint()
