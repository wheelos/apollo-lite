load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

exports_files(["openloc.MODULE.bazel"])

proto_library(
    name = "localization_config_proto",
    srcs = ["proto/localization_config.proto"],
    deps = [
        "//modules/openloc_localization:native_global_diagnostic_proto",
        "//modules/openloc_localization:native_lio_host_config_proto",
    ],
)

cc_proto_library(
    name = "localization_config_cc_proto",
    deps = [":localization_config_proto"],
)

cc_library(
    name = "localization_adapter",
    srcs = ["localization_adapter.cc"],
    hdrs = ["localization_adapter.h"],
    deps = [
        ":localization_config_cc_proto",
        "//modules/openloc_localization:native_runtime",
        "@wheelos_common//modules/common/math",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
    ],
)

cc_library(
    name = "localization_config",
    srcs = ["localization_config.cc"],
    hdrs = ["localization_config.h"],
    deps = [
        ":localization_adapter",
        "@com_google_protobuf//:protobuf",
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/map:map_selection",
        "@wheelos_common//modules/common/vehicle_calibration:registry",
        "@wheelos_transform//modules/transform:transform_query",
    ],
)

cc_library(
    name = "localization_runtime",
    srcs = ["localization_runtime.cc"],
    hdrs = ["localization_runtime.h"],
    deps = [":localization_adapter"],
)

cc_library(
    name = "localization_component_lib",
    srcs = ["localization_component.cc"],
    hdrs = ["localization_component.h"],
    deps = [
        ":localization_config",
        ":localization_runtime",
        "@core//cyber",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:transform_broadcaster",
    ],
    alwayslink = True,
)

cc_binary(
    name = "liblocalization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":localization_component_lib"],
)

cc_test(
    name = "localization_adapter_test",
    size = "small",
    srcs = ["localization_adapter_test.cc"],
    data = ["conf/localization.pb.txt"],
    deps = [
        ":localization_adapter",
        ":localization_config",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:buffer_test_peer",
    ],
)

cc_test(
    name = "localization_graph_test",
    size = "medium",
    srcs = ["localization_graph_test.cc"],
    deps = [
        ":localization_adapter",
        "//modules/openloc_localization:native_graph_test_support",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_component_test",
    size = "small",
    srcs = ["localization_component_test.cc"],
    data = ["conf/localization.pb.txt"],
    deps = [
        ":localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        "conf/localization.pb.txt",
        "dag/localization.dag",
        "launch/localization.launch",
    ],
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "localization",
    strip_prefix = strip_prefix.from_root("modules/localization"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":liblocalization_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "localization/lib",
    strip_prefix = strip_prefix.from_root("modules/localization"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
