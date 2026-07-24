load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

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
        "//modules/common/configs:vehicle_config_helper",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = ["abstract_vehicle_factory.cc"],
    hdrs = ["abstract_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/vehicle:vehicle_controller_base",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common/util:registry",
    ],
)

cc_library(
    name = "vehicle_factory_core",
    srcs = ["vehicle_factory.cc"],
    hdrs = ["vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":abstract_vehicle_factory",
        "//modules/common/util:registry",
    ],
)

config_setting(
    name = "with_ch",
    define_values = {"vehicle": "ch"},
)
config_setting(
    name = "with_devkit",
    define_values = {"vehicle": "devkit"},
)
config_setting(
    name = "with_ge3",
    define_values = {"vehicle": "ge3"},
)
config_setting(
    name = "with_gem",
    define_values = {"vehicle": "gem"},
)
config_setting(
    name = "with_lexus",
    define_values = {"vehicle": "lexus"},
)
config_setting(
    name = "with_lincoln",
    define_values = {"vehicle": "lincoln"},
)
config_setting(
    name = "with_mk_mini",
    define_values = {"vehicle": "mk_mini"},
)
config_setting(
    name = "with_neolix_edu",
    define_values = {"vehicle": "neolix_edu"},
)
config_setting(
    name = "with_transit",
    define_values = {"vehicle": "transit"},
)
config_setting(
    name = "with_wey",
    define_values = {"vehicle": "wey"},
)
config_setting(
    name = "with_yunle",
    define_values = {"vehicle": "yunle"},
)
config_setting(
    name = "with_zhongyun",
    define_values = {"vehicle": "zhongyun"},
)

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
    name = "vehicle_factory_test",
    size = "small",
    srcs = ["vehicle_factory_test.cc"],
    deps = [
        ":vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
