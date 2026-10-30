load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

exports_files([
    "conf/traffic_light.pb.txt",
    "dag/trafficlights.dag",
])

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
        "ports/inference_regions.h",
        "ports/provider_ports.h",
        "ports/result_writer_port.h",
    ],
    deps = [
        ":tl_common",
        "//modules/perception/inference/onnx_multi_batch:onnx_multi_batch_infer",
        "@core//cyber",
        "@opencv//:imgproc",
    ],
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
        "stages/tracker_prediction_stage.h",
        "stages/tracker_stage.h",
    ],
    deps = [
        ":tl_common",
        ":tl_ports",
        ":tl_stage_interface",
        "//modules/perception/common/graph:hungarian_optimizer",
        "//modules/perception/common/graph:secure_matrix",
    ],
)

cc_library(
    name = "tl_pipeline_assembly",
    hdrs = ["pipeline/traffic_light_pipeline.h"],
    deps = [
        ":tl_common",
        ":tl_pipeline",
        ":tl_ports",
        ":tl_stages",
    ],
)

cc_library(
    name = "tl_component",
    srcs = ["traffic_light_component.cc"],
    hdrs = [
        "traffic_light_component.h",
    ],
    deps = [
        ":tl_common",
        ":tl_pipeline",
        ":tl_pipeline_assembly",
        ":tl_ports",
        ":tl_stages",
        ":traffic_light_component_config_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
        "@wheelos_msgs//wheelos_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "tl_stage_test",
    size = "small",
    srcs = [
        "stages/binder_stage_test.cc",
        "stages/tracker_stage_test.cc",
    ],
    deps = [
        ":tl_common",
        ":tl_pipeline",
        ":tl_pipeline_assembly",
        ":tl_ports",
        ":tl_stages",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "openlane_replay",
    srcs = ["tools/openlane_replay.cc"],
    deps = [
        ":tl_common",
        ":tl_pipeline",
        ":tl_pipeline_assembly",
        ":tl_ports",
        ":tl_stages",
    ],
)

py_test(
    name = "export_openlane_test",
    srcs = [
        "tools/export_openlane.py",
        "tools/export_openlane_test.py",
    ],
    imports = ["tools"],
    main = "tools/export_openlane_test.py",
)

cc_binary(
    name = "libtraffic_light_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":tl_component"],
)

cpplint()
