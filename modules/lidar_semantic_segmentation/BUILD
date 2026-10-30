load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_types",
    hdrs = ["types/semantic_types.h"],
)

cc_library(
    name = "range_projection",
    srcs = ["types/range_projection.cc"],
    hdrs = ["types/range_projection.h"],
    deps = [
        ":semantic_types",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "rangeret",
    srcs = ["inference/rangeret.cc"],
    hdrs = ["inference/rangeret.h"],
    deps = [
        ":range_projection",
        ":semantic_types",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "cylinder_projection",
    srcs = ["types/cylinder_projection.cc"],
    hdrs = ["types/cylinder_projection.h"],
    deps = ["@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto"],
)

config_setting(
    name = "cylinder_spconv_enabled",
    define_values = {
        "CYLINDER3D_SPCONV": "true",
        "USE_GPU": "true",
    },
)

cc_library(
    name = "cylinder_sparse",
    srcs = ["inference/cylinder_sparse.cc"] + select({
        ":cylinder_spconv_enabled": ["inference/cylinder_spconv.cc"],
        "//conditions:default": [],
    }),
    hdrs = [
        "inference/cylinder_sparse.h",
        "inference/cylinder_spconv.h",
    ],
    local_defines = select({
        ":cylinder_spconv_enabled": ["CYLINDER3D_WITH_SPCONV=1"],
        "//conditions:default": [],
    }),
    deps = [
        ":cylinder_projection",
    ] + select({
        "//tools/platform:use_gpu": ["@libtorch_gpu"],
        "//conditions:default": ["@libtorch_cpu"],
    }) + select({
        ":cylinder_spconv_enabled": ["@spconv"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "cylinder_batch_norm",
    srcs = ["inference/cylinder_batch_norm.cc"],
    hdrs = ["inference/cylinder_batch_norm.h"],
    deps = select({
        "//tools/platform:use_gpu": [
            ":cylinder_batch_norm_cuda",
            "@libtorch_gpu",
        ],
        "//conditions:default": ["@libtorch_cpu"],
    }),
)

cuda_library(
    name = "cylinder_batch_norm_cuda",
    srcs = ["inference/cylinder_batch_norm.cu"],
    hdrs = ["inference/cylinder_batch_norm.h"],
    copts = ["-std=c++17"],
    deps = [
        "@cuda//:cuda_runtime",
        "@libtorch_gpu",
    ],
)

cc_library(
    name = "cylinder3d",
    srcs = ["inference/cylinder3d.cc"],
    hdrs = ["inference/cylinder3d.h"],
    deps = [
        ":cylinder_batch_norm",
        ":cylinder_projection",
        ":cylinder_sparse",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@com_github_nlohmann_json//:json",
    ] + select({
        "//tools/platform:use_gpu": ["@libtorch_gpu"],
        "//conditions:default": ["@libtorch_cpu"],
    }),
)

cc_library(
    name = "cylinder3d_component",
    srcs = ["component/cylinder3d_component.cc"],
    hdrs = ["component/cylinder3d_component.h"],
    deps = [
        ":cylinder3d",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcylinder3d_component_plugin.so",
    linkshared = True,
    linkstatic = False,
    deps = [":cylinder3d_component"],
)

cc_test(
    name = "cylinder3d_test",
    size = "small",
    srcs = ["inference/cylinder3d_test.cc"],
    deps = [
        ":cylinder3d",
        ":cylinder_batch_norm",
        ":cylinder_projection",
        ":cylinder_sparse",
        "@com_github_nlohmann_json//:json",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "cylinder3d_component_test",
    size = "small",
    srcs = ["component/cylinder3d_component_test.cc"],
    tags = ["manual"],
    deps = [
        ":cylinder3d_component",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
    ],
)

cc_library(
    name = "tensorrt_executor",
    srcs = ["inference/tensorrt_executor.cc"],
    hdrs = ["inference/tensorrt_executor.h"],
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":rangeret",
        "@core//cyber",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "lidar_semantic_segmentation_component",
    srcs = ["component/lidar_semantic_segmentation_component.cc"],
    hdrs = ["component/lidar_semantic_segmentation_component.h"],
    deps = [
        ":rangeret",
        ":tensorrt_executor",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "liblidar_semantic_segmentation_component_plugin.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_semantic_segmentation_component"],
)

cc_test(
    name = "range_projection_test",
    size = "small",
    srcs = ["types/range_projection_test.cc"],
    deps = [
        ":range_projection",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "project_semantickitti",
    srcs = ["tools/project_semantickitti.cc"],
    deps = [
        ":range_projection",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "evaluate_cylinder3d",
    srcs = ["tools/evaluate_cylinder3d.cc"],
    deps = [
        ":cylinder3d",
        "@com_github_nlohmann_json//:json",
    ],
)

filegroup(
    name = "runtime_config",
    srcs = glob([
        "conf/**",
        "dag/**",
    ]),
)

install(
    name = "install",
    data = [":runtime_config"],
    data_dest = "lidar_semantic_segmentation",
    library_dest = "lidar_semantic_segmentation/lib",
    targets = [
        ":libcylinder3d_component_plugin.so",
        ":liblidar_semantic_segmentation_component_plugin.so",
    ],
)

cpplint(extra_srcs = [
    "inference/cylinder_batch_norm.cu",
    "inference/cylinder_sparse.cc",
    "inference/cylinder_spconv.cc",
])
