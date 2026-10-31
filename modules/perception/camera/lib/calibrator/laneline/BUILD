load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_based_calibrator",
    srcs = ["lane_based_calibrator.cc"],
    hdrs = ["lane_based_calibrator.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":lane_struct_for_calib",
        "//modules/perception/camera/lib/calibrator/common:histogram_estimator",
        "//modules/perception/common/i_lib/core",
    ],
)

cc_library(
    name = "lane_struct_for_calib",
    hdrs = ["lane_struct_for_calib.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_library(
    name = "laneline_calibrator",
    srcs = ["laneline_calibrator.cc"],
    hdrs = ["laneline_calibrator.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":lane_based_calibrator",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/i_lib/core",
    ],
    alwayslink = True,
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
