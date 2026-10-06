load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
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
    name = "planning_runtime_context",
    hdrs = ["planning_runtime_context.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_command_cc_proto",
    ],
)

cc_library(
    name = "planning_component_lib",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_coordinator",
        "@core//cyber",
        "//modules/execution_state_sync:client",
        "//modules/execution_state_sync:execution_state_gflags",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_runtime_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:motion_execution_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:navigation_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_command_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:hybrid_maneuver_supervisor",
        "//modules/planning/common:message_process",
        "//modules/planning/common:motion_envelope",
        "//modules/planning/common:motion_plan_builder",
        "//modules/planning/common:planning_semantics",
        "//modules/planning/common:terminal_servo_guard",
        "//modules/planning/environment:capability_extractor",
        "//modules/planning/environment:environment_model_builder",
        "//modules/planning/validation:validation_supervisor",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/storytelling_msgs:story_cc_proto",
        "//modules/routing:routing_service",
    ],
    alwayslink = True,
)

cc_library(
    name = "planning_shell_registry",
    srcs = ["shells/planning_shell_registry.cc"],
    hdrs = ["shells/planning_shell_registry.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "//modules/planning/mode:mode_resolution",
    ],
)

cc_test(
    name = "planning_mission_sync_test",
    srcs = ["planning_mission_sync_test.cc"],
    deps = [
        ":planning_component_lib",
        ":planning_coordinator",
        "//modules/execution_state_sync:client",
        "//modules/execution_state_sync:execution_state_sync",
        "//modules/planning/mission_session:mission_session_manager",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_cc_proto",
    ],
)

cc_library(
    name = "planning_coordinator",
    srcs = ["planning_coordinator.cc"],
    hdrs = ["planning_coordinator.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/planning/mode:mode_resolution",
        "//modules/planning/mode:shell_transition_policy",
        "//modules/planning/mission_session:mission_session_manager",
        ":open_space_planning",
        ":on_lane_planning",
        ":planning_shell_registry",
        ":planning_shell_bridge",
        ":planning_runtime_context",
        "@wheelos_common//modules/common/status",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:local_view",
        "//modules/planning/common:published_trajectory_gear",
        "//modules/planning/environment:capability_extractor",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_library(
    name = "planning_shell_bridge",
    hdrs = ["planning_shell_bridge.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "@core//cyber",
    ],
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
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:local_view",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/environment:capability_extractor",
        ":planning_runtime_context",
        "//modules/planning/planner",
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
    name = "open_space_planning",
    srcs = ["open_space_planning.cc"],
    hdrs = ["open_space_planning.h"],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:published_trajectory_gear",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:stage",
        "//modules/planning/scenarios/park/park_and_go:park_and_go_scenario",
        "//modules/planning/scenarios/park/pull_over:pull_over_scenario",
        "//modules/planning/scenarios/park/valet_parking",
    ],
)

cc_library(
    name = "on_lane_planning",
    srcs = [
        "on_lane_debug_exporter.cc",
        "on_lane_planning.cc",
    ],
    hdrs = [
        "on_lane_debug_exporter.h",
        "on_lane_planning.h",
    ],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_base",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:published_trajectory_gear",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/planning/planner:planner_selector",
        "//modules/planning/proto:planning_semantic_map_config_cc_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/scenarios/park/valet_parking",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "planning_shell_registry_test",
    size = "small",
    srcs = ["shells/planning_shell_registry_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":planning_shell_registry",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "open_space_planning_test",
    size = "small",
    srcs = ["open_space_planning_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":open_space_planning",
        "@core//cyber/common:file",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
    linkstatic = True,
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
    ],
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
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_command_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:decision_py_pb2",
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
