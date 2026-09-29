load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

PREDICTION_FOPENMP_COPTS = [
    "-DMODULE_NAME=\\\"prediction\\\"",
    "-fopenmp",
]

cc_library(
    name = "mlp_evaluator",
    srcs = ["mlp_evaluator.cc"],
    hdrs = ["mlp_evaluator.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:fnn_vehicle_model_cc_proto",
    ],
)

cc_test(
    name = "mlp_evaluator_test",
    size = "small",
    srcs = ["mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "cost_evaluator",
    srcs = ["cost_evaluator.cc"],
    hdrs = ["cost_evaluator.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
    ],
)

cc_test(
    name = "cost_evaluator_test",
    size = "small",
    srcs = ["cost_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "junction_mlp_evaluator",
    srcs = ["junction_mlp_evaluator.cc"],
    hdrs = ["junction_mlp_evaluator.h"],
    copts = PREDICTION_FOPENMP_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_test(
    name = "junction_mlp_evaluator_test",
    size = "small",
    srcs = ["junction_mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:junction_mlp_evaluator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "junction_map_evaluator",
    srcs = ["junction_map_evaluator.cc"],
    hdrs = ["junction_map_evaluator.h"],
    copts = PREDICTION_FOPENMP_COPTS,
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_library(
    name = "cruise_mlp_evaluator",
    srcs = ["cruise_mlp_evaluator.cc"],
    hdrs = ["cruise_mlp_evaluator.h"],
    copts = PREDICTION_FOPENMP_COPTS,
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_test(
    name = "cruise_mlp_evaluator_test",
    size = "small",
    srcs = ["cruise_mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:cruise_mlp_evaluator",
    ],
)

cc_library(
    name = "lane_scanning_evaluator",
    srcs = ["lane_scanning_evaluator.cc"],
    hdrs = ["lane_scanning_evaluator.h"],
    copts = PREDICTION_FOPENMP_COPTS,
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_library(
    name = "lane_aggregating_evaluator",
    srcs = ["lane_aggregating_evaluator.cc"],
    hdrs = ["lane_aggregating_evaluator.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_library(
    name = "semantic_lstm_evaluator",
    srcs = ["semantic_lstm_evaluator.cc"],
    hdrs = ["semantic_lstm_evaluator.h"],
    copts = PREDICTION_FOPENMP_COPTS,
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
        "@eigen",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_library(
    name = "hivt_scene_feature_builder",
    srcs = ["hivt_scene_feature_builder.cc"],
    hdrs = [
        "hivt_scene.h",
        "hivt_scene_feature_builder.h",
    ],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/map/hdmap:hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
    ],
)

cc_test(
    name = "hivt_scene_feature_builder_test",
    size = "small",
    srcs = ["hivt_scene_feature_builder_test.cc"],
    data = ["//modules/prediction:prediction_testdata"],
    deps = [
        ":hivt_scene_feature_builder",
        "//modules/prediction/common:kml_map_based_test",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hivt_tensorrt_executor",
    srcs = if_gpu(
        ["hivt_tensorrt_executor_gpu.cc"],
        ["hivt_tensorrt_executor_cpu.cc"],
    ),
    hdrs = [
        "hivt_scene.h",
        "hivt_tensorrt_executor.h",
    ],
    copts = PREDICTION_COPTS,
    deps = [
        "@core//cyber",
        "//modules/prediction/common:prediction_system_gflags",
    ] + if_gpu(
        [
            "@cuda//:cuda_runtime",
            "@local_config_tensorrt//:tensorrt",
        ],
        [],
    ),
)

cc_test(
    name = "hivt_tensorrt_executor_test",
    size = "small",
    srcs = ["hivt_tensorrt_executor_test.cc"],
    tags = ["manual"],
    deps = [
        ":hivt_tensorrt_executor",
        "//modules/prediction/common:prediction_system_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hivt_scene_evaluator",
    srcs = ["hivt_scene_evaluator.cc"],
    hdrs = ["hivt_scene_evaluator.h"],
    copts = PREDICTION_COPTS,
    deps = [
        ":hivt_scene_feature_builder",
        ":hivt_tensorrt_executor",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_system_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
    ],
)

cc_test(
    name = "hivt_scene_evaluator_test",
    size = "small",
    srcs = ["hivt_scene_evaluator_test.cc"],
    deps = [
        ":hivt_scene_evaluator",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
