load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "buffer",
    srcs = [
        "buffer.cc",
    ],
    hdrs = [
        "buffer.h",
        "buffer_interface.h",
        "transform_diagnostics.h",
    ],
    deps = [
        ":buffer_interface",
        "//modules/common/adapters:adapter_gflags",
        "//third_party/tf2",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "transform_query",
    srcs = ["transform_query.cc"],
    hdrs = ["transform_query.h"],
    deps = [
        ":buffer",
        ":buffer_interface",
        "//third_party/tf2",
        "@core//cyber/common:log",
        "@core//cyber/time",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "timed_transform_resolver",
    srcs = ["timed_transform_resolver.cc"],
    hdrs = ["timed_transform_resolver.h"],
    deps = [
        ":transform_query",
        "//modules/common/util:string_util",
        "@core//cyber/common:global_data",
        "@core//cyber/common:log",
        "@core//cyber/time",
        "@eigen",
    ],
)

cc_library(
    name = "buffer_interface",
    hdrs = [
        "buffer_interface.h",
        "transform_diagnostics.h",
    ],
    deps = [
        "//third_party/tf2",
        "@core//cyber/time",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_test(
    name = "transform_query_test",
    size = "small",
    srcs = ["transform_query_test.cc"],
    deps = [
        ":buffer_interface",
        ":transform_query",
        "//third_party/tf2",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "timed_transform_resolver_test",
    size = "small",
    srcs = ["timed_transform_resolver_test.cc"],
    data = [
        "@core//cyber:cyber_conf",
    ],
    deps = [
        ":buffer_interface",
        ":timed_transform_resolver",
        ":transform_query",
        "@bazel_tools//tools/cpp/runfiles",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "transform_broadcaster",
    srcs = [
        "transform_broadcaster.cc",
    ],
    hdrs = ["transform_broadcaster.h"],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "calibration_registry",
    srcs = ["calibration_registry.cc"],
    hdrs = ["calibration_registry.h"],
    deps = [
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
    ],
)

cc_test(
    name = "calibration_registry_test",
    size = "small",
    srcs = ["calibration_registry_test.cc"],
    deps = [
        ":calibration_registry",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "static_transform_component_lib",
    srcs = ["static_transform_component.cc"],
    hdrs = ["static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    deps = [
        ":calibration_registry",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "static_transform_component_test",
    size = "small",
    srcs = ["static_transform_component_test.cc"],
    data = [
        "@core//cyber:cyber_conf",
    ],
    deps = [
        ":static_transform_component_lib",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "libstatic_transform_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":static_transform_component_lib"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "transform",
    library_dest = "transform/lib",
    targets = [
        ":libstatic_transform_component.so",
        ":buffer",
        ":calibration_registry",
        ":timed_transform_resolver",
        ":transform_query",
        ":transform_broadcaster",
    ],
    deps = [
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.yaml",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_transform_hdrs",
        ":install_transform_src",
    ],
)

install_src_files(
    name = "install_transform_src",
    dest = "transform/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_transform_hdrs",
    dest = "transform/include",
    filter = "*.h",
    src_dir = ["."],
)

cpplint()
