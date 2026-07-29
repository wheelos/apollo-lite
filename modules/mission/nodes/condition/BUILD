load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "condition",
    deps = [
        ":check_battery",
    ],
)

cc_library(
    name = "check_battery",
    srcs = ["check_battery.cc"],
    hdrs = ["check_battery.h"],
    deps = [
        "@core//cyber",
        "//modules/mission/common:mission_context",
        "@behaviortree_cpp//:behaviortree_cpp",
    ],
)

cpplint()
