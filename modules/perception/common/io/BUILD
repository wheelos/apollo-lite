load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io_util",
    srcs = ["io_util.cc"],
    hdrs = ["io_util.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:camera",
        "//modules/perception/base:distortion_model",
        "//modules/perception/base:omnidirectional_model",
        "//modules/perception/common/geometry:basic",
        "@boost.filesystem",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "brown_camera_intrinsic_test",
    size = "small",
    srcs = ["brown_camera_intrinsic_test.cc"],
    deps = [
        ":io_util",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

cc_test(
    name = "io_util_test",
    size = "small",
    srcs = ["io_util_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":io_util",
        "@com_google_googletest//:gtest_main",
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
