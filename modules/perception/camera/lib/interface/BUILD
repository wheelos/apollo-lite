load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_lane_postprocessor",
    hdrs = ["base_lane_postprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_calibrator",
    hdrs = ["base_calibrator.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:plugin",
        "@core//cyber",
    ],
)

cc_library(
    name = "base_cipv",
    hdrs = ["base_cipv.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "@core//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "base_tl_preprocessor",
    hdrs = ["base_tl_preprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base:traffic_light",
        "//modules/perception/camera/lib/traffic_light/preprocessor:pose",
        "//modules/perception/lib/registerer",
        "@core//cyber",
    ],
)

cc_library(
    name = "base_lane_detector",
    hdrs = ["base_lane_detector.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_landmark_detector",
    hdrs = ["base_landmark_detector.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_calibration_service",
    hdrs = ["base_calibration_service.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/common/util:util_tool",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_obstacle_postprocessor",
    hdrs = ["base_obstacle_postprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_obstacle_tracker",
    hdrs = ["base_obstacle_tracker.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_obstacle_transformer",
    hdrs = ["base_obstacle_transformer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_scene_parser",
    hdrs = ["base_scene_parser.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_lane_tracker",
    hdrs = ["base_lane_tracker.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_traffic_light_detector",
    hdrs = ["base_traffic_light_detector.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_init_options",
    hdrs = ["base_init_options.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
)

cc_library(
    name = "base_obstacle_detector",
    hdrs = ["base_obstacle_detector.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_traffic_light_tracker",
    hdrs = ["base_traffic_light_tracker.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_feature_extractor",
    hdrs = ["base_feature_extractor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_inference_engine",
    hdrs = ["base_inference_engine.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "base_camera_perception",
    hdrs = ["base_camera_perception.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "interface",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_calibration_service",
        ":base_calibrator",
        ":base_camera_perception",
        ":base_cipv",
        ":base_feature_extractor",
        ":base_inference_engine",
        ":base_init_options",
        ":base_landmark_detector",
        ":base_lane_detector",
        ":base_lane_postprocessor",
        ":base_lane_tracker",
        ":base_obstacle_detector",
        ":base_obstacle_postprocessor",
        ":base_obstacle_tracker",
        ":base_obstacle_transformer",
        ":base_scene_parser",
        ":base_tl_preprocessor",
        ":base_traffic_light_detector",
        ":base_traffic_light_tracker",
    ],
)

cpplint()
