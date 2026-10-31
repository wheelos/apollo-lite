load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_detection_postprocessor",
    srcs = ["camera_detection_postprocessor.cc"],
    hdrs = ["camera_detection_postprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_get_object",
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/filter_bbox",
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/recover_bbox",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:camera_detection_postprocessor_config_cc_proto",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
