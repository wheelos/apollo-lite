load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "navi_planning",
    srcs = ["navi_planning.cc"],
    hdrs = ["navi_planning.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\""
    ],
    deps = [
        ":planning_base",
        "//cyber",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/traffic_rules:traffic_decider",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
