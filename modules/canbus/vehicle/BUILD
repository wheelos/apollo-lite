load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/hardware:defs.bzl", "VEHICLES", "vehicle_selection")

package(default_visibility = ["//visibility:public"])

exports_files(["chassis_extension_tools.h"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "chassis_extension_tools",
    hdrs = ["chassis_extension_tools.h"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cc_library(
    name = "vehicle_controller_base",
    srcs = ["vehicle_controller.cc"],
    hdrs = ["vehicle_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = ["abstract_vehicle_factory.cc"],
    hdrs = ["abstract_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@wheelos_common//modules/common/util:registry",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
    ],
)

cc_library(
    name = "vehicle_factory_core",
    srcs = ["vehicle_factory.cc"],
    hdrs = ["vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":abstract_vehicle_factory",
        "@wheelos_common//modules/common/util:registry",
    ],
)

[vehicle_selection(name = "with_" + brand) for brand in VEHICLES]

cc_library(
    name = "vehicle_factory_plugins_all",
    deps = select({
        ":with_ch": ["//modules/canbus/vehicle/ch:ch_vehicle_factory"],
        ":with_devkit": ["//modules/canbus/vehicle/devkit:devkit_vehicle_factory"],
        ":with_ge3": ["//modules/canbus/vehicle/ge3:ge3_vehicle_factory"],
        ":with_gem": ["//modules/canbus/vehicle/gem:gem_vehicle_factory"],
        ":with_lexus": ["//modules/canbus/vehicle/lexus:lexus_vehicle_factory"],
        ":with_lincoln": ["//modules/canbus/vehicle/lincoln:lincoln_vehicle_factory"],
        ":with_mk_mini": ["//modules/canbus/vehicle/mk_mini:mk_mini_vehicle_factory"],
        ":with_neolix_edu": ["//modules/canbus/vehicle/neolix_edu:neolix_edu_vehicle_factory"],
        ":with_transit": ["//modules/canbus/vehicle/transit:transit_vehicle_factory"],
        ":with_wey": ["//modules/canbus/vehicle/wey:wey_vehicle_factory"],
        ":with_yunle": ["//modules/canbus/vehicle/yunle:yunle_vehicle_factory"],
        ":with_zhongyun": ["//modules/canbus/vehicle/zhongyun:zhongyun_vehicle_factory"],
        "//conditions:default": [],
    }),
    alwayslink = True,
)

cc_library(
    name = "vehicle_factory",
    deps = [
        ":vehicle_factory_core",
        ":vehicle_factory_plugins_all",
    ],
)

cc_test(
    name = "vehicle_selection_test",
    size = "small",
    srcs = ["vehicle_selection_test.cc"],
    target_compatible_with = select({
        ":with_devkit": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vehicle_factory_test",
    size = "small",
    srcs = ["vehicle_factory_test.cc"],
    deps = [
        ":vehicle_factory",
        "//modules/canbus/vehicle/ch:ch_vehicle_factory",
        "//modules/canbus/vehicle/devkit:devkit_vehicle_factory",
        "//modules/canbus/vehicle/ge3:ge3_vehicle_factory",
        "//modules/canbus/vehicle/gem:gem_vehicle_factory",
        "//modules/canbus/vehicle/lincoln:lincoln_vehicle_factory",
        "//modules/canbus/vehicle/neolix_edu:neolix_edu_vehicle_factory",
        "//modules/canbus/vehicle/wey:wey_vehicle_factory",
        "//modules/canbus/vehicle/zhongyun:zhongyun_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
