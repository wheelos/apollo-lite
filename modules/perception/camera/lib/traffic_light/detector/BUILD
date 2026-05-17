load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "detection",
    srcs = [
        "detection.cc",
        "yolo_single_stage_dector.cc",
    ],
    hdrs = [
        "detection.h",
        "yolo_single_stage_dector.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":cropbox",
        ":select",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:detection_cc_proto",
        "@opencv//:imgcodecs",
    ]  + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_library(
    name = "select",
    srcs = ["select.cc"],
    hdrs = ["select.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/graph:hungarian_optimizer",
        "//modules/perception/common/graph:secure_matrix",
    ],
)

cc_library(
    name = "cropbox",
    srcs = ["cropbox.cc"],
    hdrs = ["cropbox.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
    ],
)

cc_test(
    name = "detection_test",
    size = "small",
    srcs = ["detection_test.cc"],
    deps = [
        ":detection",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cpplint()
