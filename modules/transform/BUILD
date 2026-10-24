load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:private"])

PUBLIC_HDRS = [
    "buffer.h",
    "buffer_interface.h",
    "ground_reference.h",
    "timed_transform_resolver.h",
    "transform_broadcaster.h",
    "transform_diagnostics.h",
    "transform_query.h",
]

cc_library(
    name = "ground_reference",
    srcs = ["internal/ground_reference.cc"],
    hdrs = ["ground_reference.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":transform_query",
        "@core//cyber/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "buffer",
    srcs = [
        "internal/buffer.cc",
        "internal/buffer_impl.cc",
        "internal/buffer_impl.h",
        "internal/transform_listener.cc",
        "internal/transform_listener.h",
    ],
    hdrs = [
        "buffer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_interface",
        "//third_party/tf2",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "transform_query",
    srcs = ["internal/transform_query.cc"],
    hdrs = ["transform_query.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_interface",
        "@core//cyber/common:log",
        "@core//cyber/time",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "timed_transform_resolver",
    srcs = [
        "internal/timed_transform_resolver.cc",
        "internal/transform_cache.cc",
        "internal/transform_cache.h",
    ],
    hdrs = ["timed_transform_resolver.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":transform_query",
        "@core//cyber/common:log",
        "@core//cyber/time",
        "@eigen",
        "@wheelos_common//modules/common/util:string_util",
    ],
)

cc_library(
    name = "buffer_interface",
    hdrs = [
        "buffer_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":transform_diagnostics",
        "@core//cyber/time",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "transform_diagnostics",
    srcs = ["internal/transform_diagnostics.cc"],
    hdrs = ["transform_diagnostics.h"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_nlohmann_json//:json"],
)

cc_library(
    name = "buffer_test_peer",
    testonly = True,
    srcs = [
        "internal/buffer_impl.h",
        "internal/transform_listener.h",
        "tests/buffer_test_peer.cc",
    ],
    hdrs = ["tests/buffer_test_peer.h"],
    visibility = [
        "//modules/dreamview/backend/point_cloud:__pkg__",
        "//modules/drivers/lidar/fusion:__pkg__",
        "//modules/ndt_localization:__pkg__",
        "//modules/perception/onboard/transform_wrapper:__pkg__",
    ],
    deps = [
        ":buffer",
        "//third_party/tf2",
        "@core//cyber",
    ],
)

cc_test(
    name = "transform_query_test",
    size = "small",
    srcs = ["tests/transform_query_test.cc"],
    deps = [
        ":buffer_interface",
        ":transform_query",
        "@com_github_nlohmann_json//:json",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "timed_transform_resolver_test",
    size = "small",
    srcs = [
        "internal/transform_cache.h",
        "tests/timed_transform_resolver_test.cc",
    ],
    deps = [
        ":buffer_interface",
        ":timed_transform_resolver",
        ":transform_query",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "transform_broadcaster",
    srcs = [
        "internal/transform_broadcaster.cc",
    ],
    hdrs = ["transform_broadcaster.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "static_transform_loader",
    srcs = ["calibration/static_transform_loader.cc"],
    hdrs = ["calibration/static_transform_loader.h"],
    visibility = [
        "//modules/dreamview/backend/point_cloud:__pkg__",
        "//modules/perception/onboard/transform_wrapper:__pkg__",
        "//modules/tools/vehicle_calibration:__pkg__",
    ],
    deps = [
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber/common:log",
        "@wheelos_common//modules/common/vehicle_calibration:registry",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_test(
    name = "static_transform_loader_test",
    srcs = ["tests/static_transform_loader_test.cc"],
    deps = [
        ":static_transform_loader",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

cc_test(
    name = "buffer_static_test",
    srcs = ["tests/buffer_static_test.cc"],
    deps = [
        ":buffer",
        ":buffer_test_peer",
        ":ground_reference",
        ":transform_query",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "static_transform_component_lib",
    srcs = ["components/static_transform_component.cc"],
    hdrs = ["components/static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    deps = [
        ":static_transform_loader",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_common//modules/common/vehicle_calibration:registry",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "static_transform_component_test",
    size = "small",
    srcs = ["tests/static_transform_component_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "@core//cyber:cyber_conf",
    ],
    deps = [
        ":buffer",
        ":buffer_test_peer",
        ":static_transform_component_lib",
        "@bazel_tools//tools/cpp/runfiles",
        "@com_google_googletest//:gtest",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

cc_binary(
    name = "libstatic_transform_component.so",
    linkshared = True,
    linkstatic = False,
    visibility = ["//visibility:public"],
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
        ":ground_reference",
        ":timed_transform_resolver",
        ":transform_query",
        ":transform_broadcaster",
        ":transform_diagnostics",
    ],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
    visibility = ["//visibility:public"],
)

install(
    name = "install_src",
    visibility = ["//visibility:public"],
    deps = [
        ":install_transform_hdrs",
        ":install_transform_src",
    ],
)

install(
    name = "install_transform_src",
    data = glob([
        "**/*.cc",
        "**/*.h",
        "BUILD",
        "README.md",
        "dag/*.dag",
        "launch/*.launch",
    ]),
    data_dest = "transform/src",
)

install(
    name = "install_transform_hdrs",
    data = PUBLIC_HDRS,
    data_dest = "transform/include",
)

cpplint()
