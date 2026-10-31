load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

pkg_files(
    name = "package_files_archives",
    srcs = [
        ":evaluator_submodule_lib",
        ":prediction_lego_lib",
        ":predictor_submodule_lib",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/prediction/submodules"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "submodule_output",
    srcs = ["submodule_output.cc"],
    hdrs = ["submodule_output.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacle",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_cc_proto",
    ],
)

cc_library(
    name = "evaluator_submodule_lib",
    srcs = ["evaluator_submodule.cc"],
    hdrs = ["evaluator_submodule.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/adapters/proto:adapter_config_cc_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "predictor_submodule_lib",
    srcs = ["predictor_submodule.cc"],
    hdrs = ["predictor_submodule.h"],
    copts = PREDICTION_COPTS,
    deps = [
        ":evaluator_submodule_lib",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/adapters/proto:adapter_config_cc_proto",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/predictor:predictor_manager",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "prediction_lego_lib",
    deps = [
        ":evaluator_submodule_lib",
        ":predictor_submodule_lib",
        "//modules/prediction:prediction_component_lib",
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
