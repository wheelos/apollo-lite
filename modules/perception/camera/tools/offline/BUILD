load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":offline_obstacle_pipeline"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "perception/bin",
    strip_prefix = strip_prefix.from_root("modules/perception/camera/tools/offline"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

cc_library(
    name = "visualizer",
    srcs = ["visualizer.cc"],
    hdrs = [
        "colormap.h",
        "keycode.h",
        "visualizer.h",
    ],
    deps = [
        ":transform_server",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/common",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "//modules/perception/proto:motion_service_cc_proto",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "transform_server",
    srcs = ["transform_server.cc"],
    hdrs = ["transform_server.h"],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/io:io_util",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@eigen",
    ],
)

cc_binary(
    name = "offline_obstacle_pipeline",
    srcs = ["offline_obstacle_pipeline.cc"],
    linkopts = ["-lleveldb"],
    linkstatic = True,
    deps = [
        ":transform_server",
        ":visualizer",
        "//modules/perception/base",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/app/proto:perception_cc_proto",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "@com_google_absl//absl/strings",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
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
