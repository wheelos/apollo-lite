load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "local_sdk_test",
    size = "small",
    srcs = ["local_sdk_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@gstreamer",
    ],
)
