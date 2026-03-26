load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_algo_detector",
    srcs = [
        "algo/detector/traffic_light_detector_stage.cc",
    ],
    hdrs = [
        "algo/detector/traffic_light_detector_stage.h",
    ],
    deps = [
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/detector:detection",
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    ],
)

cc_library(
    name = "traffic_light_algo_tracker",
    srcs = [
        "algo/tracker/traffic_light_tracker_stage.cc",
    ],
    hdrs = [
        "algo/tracker/traffic_light_tracker_stage.h",
    ],
    deps = [
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
    ],
)

cc_library(
    name = "traffic_light_algo_preprocessor",
    srcs = [
        "algo/preprocessor/traffic_light_preprocessor_stage.cc",
    ],
    hdrs = [
        "algo/preprocessor/traffic_light_preprocessor_stage.h",
    ],
    deps = [
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
    ],
)

cc_library(
    name = "traffic_light_domain",
    srcs = [
        "domain/traffic_light_component_config.cc",
        "domain/traffic_light_pipeline_config.cc",
        "domain/traffic_light_result.cc",
    ],
    hdrs = [
        "domain/traffic_light_component_config.h",
        "domain/traffic_light_pipeline_config.h",
        "domain/traffic_light_result.h",
        "domain/traffic_light_runtime_state.h",
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "//modules/perception/traffic_light/proto:traffic_light_component_cc_proto",
    ],
)

cc_library(
    name = "traffic_light_application_ports",
    hdrs = [
        "application/ports/traffic_light_detector_port.h",
        "application/ports/traffic_light_tracker_port.h",
    ],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    ],
)

cc_library(
    name = "traffic_light_infra",
    srcs = [
        "infra/preprocessor_gateway.cc",
        "infra/runtime_stage_factory.cc",
        "infra/traffic_light_scene_gateway.cc",
    ],
    hdrs = [
        "infra/preprocessor_gateway.h",
        "infra/runtime_stage_factory.h",
        "infra/traffic_light_scene_gateway.h",
    ],
    deps = [
        ":traffic_light_algo_detector",
        ":traffic_light_algo_preprocessor",
        ":traffic_light_algo_tracker",
        ":traffic_light_application_ports",
        ":traffic_light_domain",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@boost.algorithm",
    ],
)

cc_library(
    name = "traffic_light_system",
    srcs = [
        "application/traffic_light_system.cc",
    ],
    hdrs = [
        "application/traffic_light_system.h",
    ],
    deps = [
        ":traffic_light_domain",
        ":traffic_light_infra",
        ":traffic_light_perception_pipeline",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/onboard/common_flags",
        "@boost.circular_buffer",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "traffic_light_perception_pipeline",
    srcs = ["application/traffic_light_perception_pipeline.cc"],
    hdrs = ["application/traffic_light_perception_pipeline.h"],
    deps = [
        ":traffic_light_application_ports",
        ":traffic_light_domain",
        ":traffic_light_infra",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "traffic_light_component_lib",
    srcs = [
        "component/traffic_light_component.cc",
    ],
    hdrs = [
        "component/traffic_light_component.h",
    ],
    deps = [
        ":traffic_light_domain",
        ":traffic_light_system",
        "//cyber",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "//modules/perception/traffic_light/proto:traffic_light_component_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "traffic_light_result_test",
    size = "small",
    srcs = ["domain/traffic_light_result_test.cc"],
    deps = [
        ":traffic_light_domain",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "traffic_light_component_config_test",
    size = "small",
    srcs = ["domain/traffic_light_component_config_test.cc"],
    deps = [
        ":traffic_light_domain",
        "//modules/perception/traffic_light/proto:traffic_light_component_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libtraffic_light_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":traffic_light_component_lib"],
)

cpplint()
