load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    data_dest = "common",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    targets = [
        ":monitor_log",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "monitor_log",
    hdrs = [
        "monitor_log_buffer.h",
        "monitor_logger.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":monitor_log_buffer",
        ":monitor_logger",
        "//modules/common/util:util_tool",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
)

cc_library(
    name = "monitor_logger",
    srcs = ["monitor_logger.cc"],
    hdrs = ["monitor_logger.h"],
    deps = [
        "//modules/common/util:util_tool",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "monitor_log_buffer",
    srcs = ["monitor_log_buffer.cc"],
    hdrs = ["monitor_log_buffer.h"],
    deps = [
        ":monitor_logger",
        "@com_google_googletest//:gtest",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "monitor_log_buffer_test",
    size = "small",
    srcs = ["monitor_log_buffer_test.cc"],
    deps = [
        ":monitor_log",
        "@com_google_googletest//:gtest_main",
    ],
)

# cc_test(
#     name = "monitor_logger_test",
#     size = "small",
#     srcs = ["monitor_logger_test.cc"],
#     deps = [
#         ":monitor_log",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
