load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "cyber/bin",
    targets = [
      ":cyber_recorder",
    ],
)

cc_binary(
    name = "cyber_recorder",
    srcs = ["main.cc"],
    linkopts = ["-pthread"],
    deps = [
        ":info",
        ":recorder_config",
        ":recorder",
        ":recoverer",
        ":spliter",
        "//cyber:init",
        "//cyber/common:file",
        "//cyber/common:time_conversion",
        "//cyber/tools/cyber_recorder/play:player",
    ],
)

cc_library(
    name = "info",
    srcs = ["info/info.cc"],
    hdrs = ["info/info.h"],
    deps = [
        "//cyber/common:time_conversion",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_message",
    ],
)

cc_library(
    name = "channel_rate_filter",
    srcs = ["record/filters/channel_rate_filter.cc"],
    hdrs = ["record/filters/channel_rate_filter.h"],
)

cc_test(
    name = "channel_rate_filter_test",
    size = "small",
    srcs = ["tests/channel_rate_filter_test.cc"],
    deps = [
        ":channel_rate_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "message_size_filter",
    srcs = ["record/filters/message_size_filter.cc"],
    hdrs = ["record/filters/message_size_filter.h"],
)

cc_test(
    name = "message_size_filter_test",
    size = "small",
    srcs = ["tests/message_size_filter_test.cc"],
    deps = [
        ":message_size_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "recorder_filter_integration_test",
    size = "medium",
    srcs = ["tests/recorder_filter_integration_test.cc"],
    deps = [
        ":channel_rate_filter",
        ":message_size_filter",
        "//cyber/record:header_builder",
        "//cyber/record:record_reader",
        "//cyber/record:record_writer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "recorder_config_integration_test",
    size = "small",
    srcs = ["tests/recorder_config_integration_test.cc"],
    deps = [
        ":recorder_config",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "recorder_pipeline_integration_test",
    size = "small",
    srcs = ["tests/recorder_pipeline_integration_test.cc"],
    deps = [
        ":policy_resolver",
        ":qos_stateful_filter",
        ":subscription_selector",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "recorder",
    srcs = ["record/core/recorder.cc"],
    hdrs = ["record/core/recorder.h"],
    deps = [
        ":channel_rate_filter",
        ":conflict_warning_emitter",
        ":message_size_filter",
        ":policy_resolver",
        ":qos_stateful_filter",
        ":recorder_config",
        ":subscription_selector",
        ":subscription_set",
        "//cyber",
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "@wheelos_fastdds//:fastdds",
    ],
)

cc_library(
    name = "recorder_config",
    srcs = ["record/core/recorder_config.cc"],
    hdrs = ["record/core/recorder_config.h"],
    deps = [
        ":channel_rate_filter",
        ":message_size_filter",
        ":policy_resolver",
        ":subscription_selector",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "subscription_selector",
    srcs = ["record/selector/subscription_selector.cc"],
    hdrs = ["record/selector/subscription_selector.h"],
    deps = [
        ":selector_match_type",
        ":topic_matcher",
    ],
)

cc_test(
    name = "subscription_selector_test",
    size = "small",
    srcs = ["tests/subscription_selector_test.cc"],
    deps = [
        ":subscription_selector",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "subscription_set",
    srcs = ["record/core/subscription_set.cc"],
    hdrs = ["record/core/subscription_set.h"],
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "policy_resolver",
    srcs = ["record/selector/policy_resolver.cc"],
    hdrs = ["record/selector/policy_resolver.h"],
    deps = [
        ":selector_match_type",
        ":topic_matcher",
    ],
)

cc_test(
    name = "policy_resolver_test",
    size = "small",
    srcs = ["tests/policy_resolver_test.cc"],
    deps = [
        ":policy_resolver",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "qos_stateful_filter",
    srcs = ["record/filters/qos_stateful_filter.cc"],
    hdrs = ["record/filters/qos_stateful_filter.h"],
    deps = [
        ":policy_resolver",
    ],
)

cc_test(
    name = "qos_stateful_filter_test",
    size = "small",
    srcs = ["tests/qos_stateful_filter_test.cc"],
    deps = [
        ":qos_stateful_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conflict_warning_emitter",
    srcs = ["record/core/conflict_warning_emitter.cc"],
    hdrs = ["record/core/conflict_warning_emitter.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "topic_matcher",
    srcs = ["record/selector/topic_matcher.cc"],
    hdrs = ["record/selector/topic_matcher.h"],
    deps = [
        ":selector_match_type",
    ],
)

cc_library(
    name = "selector_match_type",
    hdrs = ["record/selector/selector_match_type.h"],
)

cc_library(
    name = "recoverer",
    srcs = ["recover/recoverer.cc"],
    hdrs = ["recover/recoverer.h"],
    deps = [
        "//cyber/base:for_each",
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cc_library(
    name = "spliter",
    srcs = ["split/spliter.cc"],
    hdrs = ["split/spliter.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cpplint()
