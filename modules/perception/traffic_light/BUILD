load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "traffic_light_component_config_proto",
    srcs = ["proto/traffic_light_component.proto"],
)

cc_proto_library(
    name = "traffic_light_component_config_cc_proto",
    deps = [":traffic_light_component_config_proto"],
)

cc_library(
    name = "tl_common",
    hdrs = [
        "common/context.h",
        "common/types.h",
    ],
)

cc_library(
    name = "tl_stage_interface",
    hdrs = ["interface/stage.h"],
    deps = [":tl_common"],
)

cc_library(
    name = "tl_ports",
    hdrs = [
        "ports/collecting_result_writer_port.h",
        "ports/default_provider_ports.h",
        "ports/hdmap_map_provider_port.h",
        "ports/provider_ports.h",
        "ports/result_writer_port.h",
    ],
    deps = [":tl_common"],
)

cc_library(
    name = "tl_pipeline",
    hdrs = ["pipeline/pipeline.h"],
    deps = [
        ":tl_common",
        ":tl_stage_interface",
    ],
)

cc_library(
    name = "tl_stages",
    hdrs = [
        "stages/binder_stage.h",
        "stages/detector_stage.h",
        "stages/fusion_stage.h",
        "stages/heuristic_stage.h",
        "stages/prompter_stage.h",
        "stages/tracker_stage.h",
    ],
    deps = [
        ":tl_common",
        ":tl_stage_interface",
    ],
)

cc_library(
    name = "tl_component",
    srcs = ["traffic_light_component.cc"],
    hdrs = ["traffic_light_component.h"],
    deps = [
        "//cyber",
        ":tl_common",
        ":tl_pipeline",
        ":tl_ports",
        ":tl_stages",
        ":traffic_light_component_config_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
    ],
)

cc_test(
    name = "tl_stage_test",
    size = "small",
    srcs = ["stages/stage_flow_test.cc"],
    deps = [
        ":tl_common",
        ":tl_pipeline",
        ":tl_stages",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libtraffic_light_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":tl_component"],
)
