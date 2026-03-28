load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tl_common",
    hdrs = [
        "common/types.h",
        "common/context.h",
    ],
)

cc_library(
    name = "tl_stage_interface",
    hdrs = [
        "interface/stage.h",
    ],
    deps = [
        ":tl_common",
    ]
)

cc_library(
    name = "tl_ports",
    hdrs = [
        "ports/default_provider_ports.h",
        "ports/collecting_result_writer_port.h",
        "ports/provider_ports.h",
        "ports/result_writer_port.h",
        "ports/hdmap_map_provider_port.h",
    ],
    deps = [
        ":tl_common",
    ]
)

cc_library(
    name = "tl_pipeline",
    hdrs = [
        "pipeline/pipeline.h",
    ],
    deps = [
        ":tl_common",
        ":tl_stage_interface",
    ]
)

cc_library(
    name = "tl_stages",
    hdrs = [
        "stages/prompter_stage.h",
        "stages/detector_stage.h",
        "stages/binder_stage.h",
        "stages/tracker_stage.h",
        "stages/heuristic_stage.h",
        "stages/fusion_stage.h",
    ],
    deps = [
        ":tl_common",
        ":tl_stage_interface",
    ]
)

cc_library(
    name = "tl_component",
    srcs = [
        "traffic_light_component.cc",
    ],
    hdrs = [
        "traffic_light_component.h",
    ],
    deps = [
        "//cyber",
        ":tl_common",
        ":tl_pipeline",
        ":tl_ports",
        ":tl_stages",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
    ]
)

cc_binary(
    name = "libtraffic_light_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":tl_component"],
)
