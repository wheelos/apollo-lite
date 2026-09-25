load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_localization_component_lib",
    srcs = [
        "rtk_localization.cc",
        "rtk_localization_component.cc",
    ],
    hdrs = [
        "rtk_localization.h",
        "rtk_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/rtk_localization/proto:rtk_config_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@com_google_googletest//:gtest",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:gps_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "librtk_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":rtk_localization_component_lib"],
)

cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = ["rtk_localization_test.cc"],
    linkstatic = True,
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//modules/common/util",
        ":rtk_localization_component_lib",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        "dag/dag_streaming_rtk_localization.dag",
        "launch/rtk_localization.launch",
        "conf/rtk_localization.pb.txt",
    ],
)

install(
    name = "install",
    data = [":runtime_data"],
    data_dest = "rtk_localization",
    library_dest = "rtk_localization/lib",
    targets = [
        ":librtk_localization_component.so",
    ],
)

cpplint()
