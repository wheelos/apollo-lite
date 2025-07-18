load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_datachecker_server",
    srcs = ["main.cc"],
    deps = [
        ":worker",
        "//cyber",
    ],
)

cc_library(
    name = "channel_verify",
    srcs = ["channel_verify.cc"],
    hdrs = ["channel_verify.h"],
    deps = [
        ":common",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "@boost.algorithm",
        "@boost.filesystem",
    ],
)

cc_library(
    name = "channel_verify_agent",
    srcs = ["channel_verify_agent.cc"],
    hdrs = ["channel_verify_agent.h"],
    deps = [
        ":channel_verify",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "common",
    srcs = ["common.cc"],
    hdrs = ["common.h"],
    deps = [
        "//cyber",
        "@boost.filesystem",
        "@boost.property_tree",
    ],
)

cc_library(
    name = "eight_route",
    srcs = ["eight_route.cc"],
    hdrs = [
        "alignment.h",
        "eight_route.h",
    ],
    deps = [
        ":common",
        ":worker_gflags",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "laps_checker",
    srcs = ["laps_checker.cc"],
    hdrs = ["laps_checker.h"],
    deps = [
        ":common",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "@boost.property_tree",
    ],
)

cc_library(
    name = "loops_verify_agent",
    srcs = ["loops_verify_agent.cc"],
    hdrs = ["loops_verify_agent.h"],
    deps = [
        ":laps_checker",
        ":pose_collection_agent",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "pj_transformer",
    srcs = ["pj_transformer.cc"],
    hdrs = ["pj_transformer.h"],
    deps = [
        "//cyber",
        "@proj",
    ],
)

cc_library(
    name = "pose_collection",
    srcs = ["pose_collection.cc"],
    hdrs = ["pose_collection.h"],
    deps = [
        ":common",
    ],
)

cc_library(
    name = "pose_collection_agent",
    srcs = ["pose_collection_agent.cc"],
    hdrs = ["pose_collection_agent.h"],
    deps = [
        ":common",
        ":pj_transformer",
        ":pose_collection",
        "//cyber",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
    ],
)

cc_library(
    name = "static_align",
    srcs = ["static_align.cc"],
    hdrs = [
        "alignment.h",
        "static_align.h",
    ],
    deps = [
        ":common",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
    ],
)

cc_library(
    name = "worker",
    srcs = ["worker.cc"],
    hdrs = ["worker.h"],
    deps = [
        ":eight_route",
        ":static_align",
        ":worker_agent",
        ":worker_cyber_node",
        ":worker_gflags",
        "//cyber",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "worker_agent",
    srcs = ["worker_agent.cc"],
    hdrs = [
        "alignment_agent.h",
        "worker_agent.h",
    ],
    deps = [
        ":channel_verify_agent",
        ":common",
        ":eight_route",
        ":loops_verify_agent",
        ":pose_collection",
        ":static_align",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "worker_cyber_node",
    srcs = ["worker_cyber_node.cc"],
    hdrs = ["worker_cyber_node.h"],
    deps = [
        ":worker_agent",
        ":worker_gflags",
        "//cyber",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
    ],
)

cc_library(
    name = "worker_gflags",
    srcs = ["worker_gflags.cc"],
    hdrs = ["worker_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
