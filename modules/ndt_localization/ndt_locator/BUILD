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
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/localization/common:localization_gflags",
        "//modules/ndt_localization/map_support/ndt_map",
        "//modules/ndt_localization/map_support/ndt_map:ndt_map_pool",
        "//modules/ndt_localization/map_support/util",
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
        "//modules/localization/common/pointcloud_io:common_io",
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
        "//modules/localization/common/pointcloud_io:common_io",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cpplint()
