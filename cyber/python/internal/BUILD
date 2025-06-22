load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "_cyber_wrapper.so",
    srcs = ["py_cyber.cc"],
    linkshared = True,
    deps = [
        ":py_cyber",
    ],
)

cc_library(
    name = "py_cyber",
    srcs = ["py_cyber.cc"],
    hdrs = ["py_cyber.h"],
    deps = [
        "//cyber:cyber_core",
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)

cc_test(
    name = "py_cyber_test",
    size = "small",
    srcs = ["py_cyber_test.cc"],
    deps = [
        ":py_cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "_cyber_record_wrapper.so",
    srcs = ["py_record.cc"],
    linkshared = True,
    deps = [
        ":py_record",
    ],
)

cc_library(
    name = "py_record",
    srcs = ["py_record.cc"],
    hdrs = ["py_record.h"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/message:py_message",
        "//cyber/record",
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)

cc_test(
    name = "py_record_test",
    size = "small",
    srcs = ["py_record_test.cc"],
    deps = [
        ":py_record",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "_cyber_time_wrapper.so",
    srcs = ["py_time.cc"],
    linkshared = True,
    deps = [
        ":py_time",
    ],
)

cc_library(
    name = "py_time",
    srcs = ["py_time.cc"],
    hdrs = ["py_time.h"],
    deps = [
        "//cyber:cyber_core",
        "@fastrtps",
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)

cc_binary(
    name = "_cyber_timer_wrapper.so",
    srcs = ["py_timer.cc"],
    linkshared = True,
    deps = [
        ":py_timer",
    ],
)

cc_library(
    name = "py_timer",
    srcs = ["py_timer.cc"],
    hdrs = ["py_timer.h"],
    deps = [
        "//cyber:cyber_core",
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)

cc_binary(
    name = "_cyber_parameter_wrapper.so",
    srcs = ["py_parameter.cc"],
    linkshared = True,
    deps = [
        ":py_parameter",
    ],
)

cc_library(
    name = "py_parameter",
    srcs = ["py_parameter.cc"],
    hdrs = ["py_parameter.h"],
    deps = [
        ":py_cyber",
        "//cyber:cyber_core",
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)

cpplint()
