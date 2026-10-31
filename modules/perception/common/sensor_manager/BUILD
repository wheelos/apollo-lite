load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_manager",
    srcs = ["sensor_manager.cc"],
    hdrs = ["sensor_manager.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:base_type",
        "//modules/perception/base:camera",
        "//modules/perception/base:omnidirectional_model",
        "//modules/perception/base:polynomial",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/thread",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/proto:sensor_meta_schema_cc_proto",
        "@core//cyber",
        "@wheelos_common//modules/common/vehicle_calibration:registry",
    ],
)

cc_test(
    name = "sensor_manager_test",
    size = "small",
    srcs = ["sensor_manager_test.cc"],
    deps = [
        ":sensor_manager",
        "@com_google_googletest//:gtest_main",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
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
