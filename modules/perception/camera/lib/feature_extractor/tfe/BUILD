load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "external_feature_extractor",
    srcs = ["external_feature_extractor.cc"],
    hdrs = ["external_feature_extractor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "@core//cyber",
    ],
)

cc_library(
    name = "project_feature",
    srcs = ["project_feature.cc"],
    hdrs = ["project_feature.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "@core//cyber",
    ],
)

cc_library(
    name = "tracking_feat_extractor",
    srcs = ["tracking_feat_extractor.cc"],
    hdrs = ["tracking_feat_extractor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference/operators:perception_inference_operators",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "@core//cyber",
    ],
    alwayslink = True,
)

cpplint()
