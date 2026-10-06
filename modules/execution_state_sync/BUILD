load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "execution_state_sync",
    srcs = ["store.cc"],
    hdrs = ["store.h"],
    deps = ["@sqlite3//:sqlite3"],
)

cc_library(
    name = "worker",
    srcs = ["worker.cc"],
    hdrs = ["worker.h"],
    deps = [":execution_state_sync"],
)

cc_library(
    name = "client",
    srcs = ["client.cc"],
    hdrs = ["client.h"],
    deps = [":worker"],
)

cc_library(
    name = "execution_state_gflags",
    srcs = ["execution_state_gflags.cc"],
    hdrs = ["execution_state_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

cc_test(
    name = "store_test",
    srcs = ["store_test.cc"],
    deps = [
        ":client",
        ":execution_state_sync",
        ":worker",
        "@com_google_googletest//:gtest_main",
        "@sqlite3//:sqlite3",
    ],
)

cpplint()
