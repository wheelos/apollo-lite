load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

pkg_files(
    name = "prediction_testdata_install_data",
    srcs = [":prediction_testdata"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "prediction/addition_data",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_filegroup(
    name = "prediction_testdata_install",
    srcs = [":prediction_testdata_install_data"],
)

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/prediction/common:message_process",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/submodules:evaluator_submodule_lib",
        "//modules/prediction/submodules:predictor_submodule_lib",
        "//modules/prediction/submodules:submodule_output",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_test(
    name = "prediction_component_test",
    size = "small",
    srcs = ["prediction_component_test.cc"],
    data = [
        ":prediction_conf",
        ":prediction_data",
        ":prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        ":prediction_component_lib",
    ],
)

cc_binary(
    name = "libprediction_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":prediction_component_lib"],
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":prediction.BUILD",
        ":runtime_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "prediction",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libprediction_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "prediction/lib",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        ":pb_hdrs",
        ":prediction_testdata_install",
        "//modules/prediction/common:package_files",
        "//modules/prediction/pipeline:package_files",
        "//modules/prediction/submodules:package_files",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = [
        "//modules/prediction/proto:fnn_model_base_cc_proto",
        "//modules/prediction/proto:fnn_vehicle_model_cc_proto",
        "//modules/prediction/proto:network_layers_cc_proto",
        "//modules/prediction/proto:network_model_cc_proto",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/proto:prediction_conf_cc_proto",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "prediction/include",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_prediction_hdrs",
        ":package_prediction_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_prediction_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/prediction/common:exported_source_files",
        "//modules/prediction/container:exported_source_files",
        "//modules/prediction/container/adc_trajectory:exported_source_files",
        "//modules/prediction/container/obstacles:exported_source_files",
        "//modules/prediction/container/pose:exported_source_files",
        "//modules/prediction/container/storytelling:exported_source_files",
        "//modules/prediction/evaluator:exported_source_files",
        "//modules/prediction/evaluator/cyclist:exported_source_files",
        "//modules/prediction/evaluator/pedestrian:exported_source_files",
        "//modules/prediction/evaluator/vehicle:exported_source_files",
        "//modules/prediction/network:exported_source_files",
        "//modules/prediction/network/rnn_model:exported_source_files",
        "//modules/prediction/pipeline:exported_source_files",
        "//modules/prediction/predictor:exported_source_files",
        "//modules/prediction/predictor/empty:exported_source_files",
        "//modules/prediction/predictor/extrapolation:exported_source_files",
        "//modules/prediction/predictor/free_move:exported_source_files",
        "//modules/prediction/predictor/interaction:exported_source_files",
        "//modules/prediction/predictor/junction:exported_source_files",
        "//modules/prediction/predictor/lane_sequence:exported_source_files",
        "//modules/prediction/predictor/move_sequence:exported_source_files",
        "//modules/prediction/predictor/sequence:exported_source_files",
        "//modules/prediction/predictor/single_lane:exported_source_files",
        "//modules/prediction/proto:exported_source_files",
        "//modules/prediction/scenario:exported_source_files",
        "//modules/prediction/scenario/analyzer:exported_source_files",
        "//modules/prediction/scenario/feature_extractor:exported_source_files",
        "//modules/prediction/scenario/interaction_filter:exported_source_files",
        "//modules/prediction/scenario/prioritization:exported_source_files",
        "//modules/prediction/scenario/right_of_way:exported_source_files",
        "//modules/prediction/scenario/scenario_features:exported_source_files",
        "//modules/prediction/submodules:exported_source_files",
        "//modules/prediction/tools:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "prediction/src",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_filegroup(
    name = "package_prediction_src",
    srcs = [":package_prediction_src_data"],
)

pkg_files(
    name = "package_prediction_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/prediction/common:exported_source_headers",
        "//modules/prediction/container:exported_source_headers",
        "//modules/prediction/container/adc_trajectory:exported_source_headers",
        "//modules/prediction/container/obstacles:exported_source_headers",
        "//modules/prediction/container/pose:exported_source_headers",
        "//modules/prediction/container/storytelling:exported_source_headers",
        "//modules/prediction/evaluator:exported_source_headers",
        "//modules/prediction/evaluator/cyclist:exported_source_headers",
        "//modules/prediction/evaluator/pedestrian:exported_source_headers",
        "//modules/prediction/evaluator/vehicle:exported_source_headers",
        "//modules/prediction/network:exported_source_headers",
        "//modules/prediction/network/rnn_model:exported_source_headers",
        "//modules/prediction/predictor:exported_source_headers",
        "//modules/prediction/predictor/empty:exported_source_headers",
        "//modules/prediction/predictor/extrapolation:exported_source_headers",
        "//modules/prediction/predictor/free_move:exported_source_headers",
        "//modules/prediction/predictor/interaction:exported_source_headers",
        "//modules/prediction/predictor/junction:exported_source_headers",
        "//modules/prediction/predictor/lane_sequence:exported_source_headers",
        "//modules/prediction/predictor/move_sequence:exported_source_headers",
        "//modules/prediction/predictor/sequence:exported_source_headers",
        "//modules/prediction/predictor/single_lane:exported_source_headers",
        "//modules/prediction/scenario:exported_source_headers",
        "//modules/prediction/scenario/analyzer:exported_source_headers",
        "//modules/prediction/scenario/feature_extractor:exported_source_headers",
        "//modules/prediction/scenario/interaction_filter:exported_source_headers",
        "//modules/prediction/scenario/prioritization:exported_source_headers",
        "//modules/prediction/scenario/right_of_way:exported_source_headers",
        "//modules/prediction/scenario/scenario_features:exported_source_headers",
        "//modules/prediction/submodules:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "prediction/include",
    strip_prefix = strip_prefix.from_root("modules/prediction"),
)

pkg_filegroup(
    name = "package_prediction_hdrs",
    srcs = [":package_prediction_hdrs_data"],
)

filegroup(
    name = "runtime_data",
    srcs = [
        ":prediction_conf",
        ":prediction_data",
    ] + glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

filegroup(
    name = "prediction_data",
    srcs = glob(
        [
            "data/*.engine",
            "data/*.pt",
            "data/*.bin",
        ],
        allow_empty = True,
    ),
)

filegroup(
    name = "prediction_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
    ]),
)

filegroup(
    name = "prediction_testdata",
    srcs = glob(["testdata/**"]),
)
