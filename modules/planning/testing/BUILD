load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gtest_exit_main",
    srcs = ["gtest_exit_main.cc"],
    deps = ["@com_google_googletest//:gtest"],
)

cpplint()
