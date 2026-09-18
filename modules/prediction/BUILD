load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

install(
    name = "prediction_testdata_install",
    data = [":prediction_testdata"],
    data_dest = "prediction/addition_data",
)

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
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

install(
    name = "install",
    data = [
        ":prediction.BUILD",
        ":runtime_data",
    ],
    data_dest = "prediction",
    library_dest = "prediction/lib",
    targets = [
        ":libprediction_component.so",
    ],
    deps = [
        ":pb_hdrs",
        ":prediction_testdata_install",
        "//modules/prediction/common:install",
        "//modules/prediction/pipeline:install",
        "//modules/prediction/submodules:install",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/prediction/proto:fnn_model_base_cc_proto",
        "//modules/prediction/proto:fnn_vehicle_model_cc_proto",
        "//modules/prediction/proto:network_layers_cc_proto",
        "//modules/prediction/proto:network_model_cc_proto",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction/proto:vector_net_cc_proto",
    ],
    data_dest = "prediction/include",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_prediction_hdrs",
        ":install_prediction_src",
    ],
)

install_src_files(
    name = "install_prediction_src",
    dest = "prediction/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_prediction_hdrs",
    dest = "prediction/include",
    filter = "*.h",
    src_dir = ["."],
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
    srcs = glob([
        "data/*.pt",
        "data/*.bin",
    ]),
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

cpplint()
