package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "caddn_obstacle_detector",
#     srcs = ["caddn_obstacle_detector.cc"],
#     hdrs = ["caddn_obstacle_detector.h"],
#     deps = [
#         "//cyber/common:file",
#         "@wheelos_common//modules/common/util:perf_util",
#         "//modules/perception/base",
#         "//modules/perception/common:perception_gflags",
#         "//modules/perception/camera/common",
#         "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
#         "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
#         "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
#         "//modules/perception/camera/lib/interface",
#         "//modules/perception/inference:inference_factory",
#         "//modules/perception/inference:inference_lib",
#         "//modules/perception/inference/utils:inference_resize_lib",
#         "//modules/perception/inference/utils:inference_util_lib",
#         "@eigen",
#         "@opencv//:imgcodecs",
#         "@opencv//:imgproc",
#         "@opencv//:highgui",
#     ],
# )

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
