load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "buffer",
    srcs = [
        "buffer.cc",
    ],
    hdrs = [
        "buffer.h",
        "buffer_interface.h",
    ],
    deps = [
        ":buffer_interface",
        "//cyber",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//third_party/tf2",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "buffer_interface",
    hdrs = ["buffer_interface.h"],
    deps = [
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "transform_broadcaster",
    srcs = [
        "transform_broadcaster.cc",
    ],
    hdrs = ["transform_broadcaster.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
    ],
)

cc_library(
    name = "static_transform_component_lib",
    srcs = ["static_transform_component.cc"],
    hdrs = ["static_transform_component.h"],
    copts = ['-DMODULE_NAME=\\"static_transform\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/transform/proto:static_transform_conf_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_test(
    name = "static_transform_component_test",
    size = "small",
    srcs = ["static_transform_component_test.cc"],
    deps = [
        ":static_transform_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libstatic_transform_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":static_transform_component_lib"],
)

install(
    name = "install",
    data_dest = "transform",
    library_dest = "transform/lib",
    data = [
        ":runtime_data",
        "transform.BUILD",
    ],
    targets = [
        ":libstatic_transform_component.so",
        ":buffer",
        ":transform_broadcaster",
    ],
    deps = [
    ]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_transform_src",
        ":install_transform_hdrs"
    ],
)

install_src_files(
    name = "install_transform_src",
    src_dir = ["."],
    dest = "transform/src",
    filter = "*",
)

install_src_files(
    name = "install_transform_hdrs",
    src_dir = ["."],
    dest = "transform/include",
    filter = "*.h",
)

cpplint()
