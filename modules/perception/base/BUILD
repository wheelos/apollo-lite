load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_type",
        ":blob",
        ":box",
        ":camera",
        ":common",
        ":distortion_model",
        ":frame",
        ":image",
        ":impending_collision_edge",
        ":object",
        ":object_pool",
        ":object_pool_types",
        ":omnidirectional_model",
        ":point_cloud",
        ":point_cloud_util",
        ":polynomial",
        ":syncedmem",
        ":traffic_light",
    ],
)

cc_library(
    name = "base_type",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = [
        "hdmap_struct.h",
        "landmark_struct.h",
        "lane_struct.h",
        "sensor_meta.h",
        "vehicle_struct.h",
    ],
    deps = [
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "blob",
    srcs = ["blob.cc"],
    hdrs = ["blob.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":common",
        ":syncedmem",
        "//cyber",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "blob_test",
    size = "small",
    srcs = ["blob_test.cc"],
    deps = [
        ":blob",
        "//modules/perception/base/test:test_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "box",
    hdrs = [
        "box.h",
        "comparison_traits.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":point_cloud",
    ],
)

cc_test(
    name = "box_test",
    size = "small",
    srcs = ["box_test.cc"],
    linkstatic = True,
    deps = [
        ":box",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "camera",
    srcs = ["camera.cc"],
    hdrs = ["camera.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "camera_test",
    size = "small",
    srcs = ["camera_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":camera",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
    ] + if_gpu([
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
    ]),
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = ["common_test.cc"],
    deps = [
        ":common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "distortion_model",
    srcs = ["distortion_model.cc"],
    hdrs = ["distortion_model.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":camera",
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "distortion_model_test",
    size = "small",
    srcs = ["distortion_model_test.cc"],
    deps = [
        ":distortion_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "frame",
    hdrs = [
        "frame.h",
        "frame_supplement.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_type",
        ":blob",
        ":image",
        ":impending_collision_edge",
        "@eigen",
    ],
)

cc_library(
    name = "image",
    hdrs = [
        "image.h",
        "image_8u.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_type",
        ":blob",
    ],
)

cc_test(
    name = "image_test",
    size = "small",
    linkstatic = True,
    deps = [
        ":image",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "impending_collision_edge",
    hdrs = ["impending_collision_edge.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_test(
    name = "impending_collision_edge_test",
    size = "small",
    srcs = ["impending_collision_edge_test.cc"],
    deps = [
        ":impending_collision_edge",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "object",
    srcs = ["object.cc"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = [
        "object.h",
        "object_supplement.h",
        "object_types.h",
        "vehicle_struct.h",
    ],
    deps = [
        ":box",
        ":point_cloud",
        "//modules/common/util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_point_cc_proto",
        "@boost.circular_buffer",
        "@eigen",
    ],
)

cc_test(
    name = "object_test",
    size = "small",
    srcs = ["object_test.cc"],
    deps = [
        ":object",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "object_pool",
    hdrs = [
        "concurrent_object_pool.h",
        "light_object_pool.h",
        "object_pool.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":base_type",
    ],
)

cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = ["object_pool_test.cc"],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":object_pool_types",
        ":point_cloud",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "object_pool_types",
    srcs = ["object_pool_types.cc"],
    hdrs = ["object_pool_types.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":point_cloud",
        "//cyber",
    ],
)

cc_library(
    name = "omnidirectional_model",
    srcs = ["omnidirectional_model.cc"],
    hdrs = ["omnidirectional_model.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":camera",
        ":distortion_model",
        ":polynomial",
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "omnidirectional_model_test",
    size = "small",
    srcs = ["omnidirectional_model_test.cc"],
    deps = [
        ":omnidirectional_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "point",
    hdrs = ["point.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
)

cc_library(
    name = "point_cloud",
    hdrs = ["point_cloud.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":point",
        "//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_library(
    name = "point_cloud_util",
    srcs = ["point_cloud_util.cc"],
    hdrs = ["point_cloud_util.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":point_cloud",
        "//cyber",
        "@eigen",
        "@opencv//:imgproc",
    ],
)

cc_test(
    name = "point_cloud_test",
    size = "small",
    srcs = ["point_cloud_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":point_cloud",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = ["polynomial.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
)

cc_test(
    name = "polynomial_test",
    size = "small",
    srcs = ["polynomial_test.cc"],
    deps = [
        ":polynomial",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "syncedmem",
    srcs = ["syncedmem.cc"],
    hdrs = ["syncedmem.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":common",
        "//cyber",
    ],
)

cc_test(
    name = "syncedmem_test",
    size = "small",
    srcs = ["syncedmem_test.cc"],
    deps = [
        ":syncedmem",
        "//modules/perception/base/test:test_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "traffic_light",
    hdrs = ["traffic_light.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:box",
    ],
)

cpplint()
