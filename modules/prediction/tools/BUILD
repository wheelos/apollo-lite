load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "free_move_record_eval",
    srcs = ["free_move_record_eval.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/math",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "@core//cyber",
        "@core//cyber/record:record_reader",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_cc_proto",
    ],
)

cpplint()
