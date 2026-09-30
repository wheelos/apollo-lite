load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "fixtures",
    srcs = glob(["data/**"]),
)

cc_library(
    name = "test_support",
    testonly = True,
    hdrs = ["test_support.h"],
    data = [":fixtures"],
    deps = [
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/fusion/base:track",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "fusion_modules_test",
    srcs = ["fusion_modules_test.cc"],
    deps = [
        ":test_support",
        "//modules/perception/fusion/base:object_snapshot",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/common:kalman_filter",
        "//modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion",
        "//modules/perception/fusion/lib/data_association/hm_data_association:hm_tracks_objects_match",
        "//modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion",
        "//modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fusion_pipeline_test",
    srcs = ["fusion_pipeline_test.cc"],
    data = ["//modules/perception/pipeline:runtime_data"],
    deps = [
        ":test_support",
        "//modules/perception/fusion/app:fusion_runtime",
        "//modules/perception/fusion/lib/data_fusion/all_latest_fusion",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/gatekeeper:collect_fused_object",
        "//modules/perception/onboard/msg_serializer",
        "@com_google_googletest//:gtest_main",
    ],
)
