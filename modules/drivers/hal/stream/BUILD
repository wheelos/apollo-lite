load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "@core//cyber",
    ],
)

cc_library(
    name = "can_stream",
    srcs = ["can_stream.cc"],
    hdrs = ["can_stream.h"],
    deps = [
        ":stream",
        "@core//cyber",
    ],
)

cc_library(
    name = "ntrip_stream",
    srcs = ["ntrip_stream.cc"],
    hdrs = ["ntrip_stream.h"],
    deps = [
        ":stream",
        ":tcp_stream",
        "@core//cyber",
        "//modules/common/util",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    hdrs = ["serial_stream.h"],
    deps = [
        ":stream",
        "@core//cyber",
    ],
)

cc_library(
    name = "tcp_stream",
    srcs = ["tcp_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "@core//cyber",
    ],
)

cc_library(
    name = "udp_stream",
    srcs = ["udp_stream.cc"],
    hdrs = ["udp_stream.h"],
    deps = [
        ":stream",
        "@core//cyber",
    ],
)

cc_library(
    name = "stream_factory",
    srcs = ["stream_factory.cc"],
    hdrs = ["stream_factory.h"],
    deps = [
        ":stream",
        ":can_stream",
        ":ntrip_stream",
        ":serial_stream",
        ":tcp_stream",
        ":udp_stream",
        "@core//cyber",
    ],
    alwayslink = 1,
)

cc_test(
    name = "udp_stream_test",
    srcs = ["udp_stream_test.cc"],
    deps = [
        ":stream_factory",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tcp_stream_test",
    srcs = ["tcp_stream_test.cc"],
    deps = [
        ":stream_factory",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

# TODO(zero): recover the test code for stream
# cc_test(
#     name = "can_stream_test",
#     srcs = ["can_stream_test.cc"],
#     deps = [
#         ":can_stream",
#         "@com_google_googletest//:gtest",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
