package(default_visibility = ["//visibility:public"])

#cc_library(
#	name = "bev_obstacle_detector",
#	srcs = ["bev_obstacle_detector.cc"],
#	hdrs = ["bev_obstacle_detector.h"],
#	deps = [
#		"@core//cyber",
#		"@wheelos_common//modules/common/util:util_tool",
#		"//modules/perception/base",
#		"//modules/perception/camera/common",
#		"//modules/perception/camera/lib/interface",
#		"//modules/perception/common:perception_gflags",
#		"//modules/perception/inference:inference_factory",
#		"//modules/perception/inference:inference_lib",
#		"//modules/perception/pipeline:stage",
#		"//modules/perception/pipeline/proto/stage:bev_obstacle_detection_config_cc_proto",
#		"@opencv//:imgproc",
#	],
#	alwayslink = True,
#)

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
