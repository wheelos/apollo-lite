load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

install(
    name = "planning_testdata_install",
    data = [":planning_testdata"],
    data_dest = if_gpu(
        "planning-gpu/addition_data",
        "planning/addition_data",
    ),
)

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":planning_component_lib"],
)

cc_library(
    name = "planning_component_lib",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":on_lane_planning",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/planning/common:history",
        "//modules/planning/common:message_process",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/storytelling_msgs:story_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:local_view",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/planner",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task_factory",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:chart_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_library(
    name = "on_lane_planning",
    srcs = ["on_lane_planning.cc"],
    hdrs = ["on_lane_planning.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_semantic_map_config_cc_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/scenarios/park/valet_parking",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ] + if_gpu(
        ["planning-gpu.BUILD"],
        ["planning.BUILD"],
    ),
    data_dest = if_gpu(
        "planning-gpu",
        "planning",
    ),
    library_dest = if_gpu(
        "planning-gpu/lib",
        "planning/lib",
    ),
    targets = [
        ":libplanning_component.so",
    ],
    deps = [
        ":planning_testdata_install",
    ],
)

install_files(
    name = "pb_planning",
    dest = if_gpu(
        "planning-gpu",
        "planning",
    ),
    files = [
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:decision_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:sl_boundary_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_planning_hdrs",
        ":install_planning_src",
    ],
)

install_src_files(
    name = "install_planning_src",
    dest = if_gpu(
        "planning-gpu/src",
        "planning/src",
    ),
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_planning_hdrs",
    dest = if_gpu(
        "planning-gpu/include",
        "planning/include",
    ),
    filter = "*.h",
    src_dir = ["."],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/planning/proto:auto_tuning_model_input_py_pb2",
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/planning/proto:open_space_task_config_py_pb2",
        "//modules/planning/proto:planner_open_space_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:task_config_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_py_pb2",
    ],
    data_dest = "planning/include",
)

cpplint()
