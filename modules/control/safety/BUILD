load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PREPROCESS_SUB_COPTS = ["-DMODULE_NAME=\\\"control\\\""]


cc_library(
    name = "safety_defines",
    hdrs = ["safety_defines.h"]
)

cc_library(
    name = "safety_manager",
    srcs = ["safety_manager.cc"],
    hdrs = ["safety_manager.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
      ":safety_defines",
      "//cyber",
      "//modules/common/util:debouncer",
      "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
      "//modules/control/proto:control_conf_cc_proto",
      "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
      "//modules/control/proto:local_view_cc_proto",
      "//modules/control/proto:safety_status_cc_proto",
      "//modules/common/configs:vehicle_config_helper",
      "@boost.container",
    ]
)

cc_test(
    name = "safety_manager_test",
    size = "small",
    srcs = ["safety_manager_test.cc"],
    deps = [
        ":safety_manager",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
