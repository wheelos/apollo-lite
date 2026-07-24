load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility=["//visibility:public"])

install(
    name="install",
    runtime_dest="drivers/bin",
    targets=[
        ":can_client_tool",
    ],
)

cc_library(
    name="can_client_factory",
    srcs=["can_client_factory.cc"],
    hdrs=["can_client_factory.h"],
    deps=[
        "//cyber",
        "//modules/common/util:util_tool",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/common:canbus_common",
    ],
    alwayslink=True,
)

cc_library(
    name="can_client",
    hdrs=["can_client.h"],
    deps=[
        "//modules/common/util",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/common:canbus_common",
    ],
    alwayslink=True,
)

cc_binary(
    name="can_client_tool",
    srcs=["can_client_tool.cc"],
    deps=[
        "//cyber",
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name="can_client_factory_test",
    size="small",
    srcs=["can_client_factory_test.cc"],
    deps=[
        "@wheelos_msgs//wheelos_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
