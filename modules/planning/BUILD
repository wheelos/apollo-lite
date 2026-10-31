load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "planning_testdata_install_data",
    srcs = [":planning_testdata"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = if_gpu("planning-gpu/addition_data", "planning/addition_data"),
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_filegroup(
    name = "planning_testdata_install",
    srcs = [":planning_testdata_install_data"],
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
    hdrs = [
        "planning_component.h",
        "planning_cycle_diagnostics.h",
        "planning_execution_state_transport.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_coordinator",
        ":planning_cycle_diagnostics",
        ":planning_cycle_result",
        ":planning_execution_state_transport",
        ":planning_runtime_status_builder",
        "@core//cyber",
        "@wheelos_common//execution_state:flags",
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
    name = "planning_cycle_diagnostics",
    srcs = ["planning_cycle_diagnostics.cc"],
    hdrs = ["planning_cycle_diagnostics.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/planning/common:hybrid_maneuver_supervisor",
        "//modules/planning/common:planning_semantics",
        ":planning_runtime_context",
    ],
)

cc_library(
    name = "planning_cycle_result",
    hdrs = ["planning_cycle_result.h"],
    deps = [
        "//modules/planning/common:hybrid_maneuver_supervisor",
        "//modules/planning/common:planning_semantics",
        ":planning_runtime_context",
        "//modules/planning/validation:validation_supervisor",
    ],
)

cc_library(
    name = "planning_execution_state_transport",
    srcs = ["planning_execution_state_transport.cc"],
    hdrs = ["planning_execution_state_transport.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "@wheelos_common//execution_state:client",
    ],
)

cc_library(
    name = "planning_runtime_status_builder",
    srcs = ["planning_runtime_status_builder.cc"],
    hdrs = ["planning_runtime_status_builder.h"],
    deps = [
        "@wheelos_common//modules/common/util:message_util",
        "//modules/planning/common:hybrid_maneuver_supervisor",
        "//modules/planning/common:planning_semantics",
        "//modules/planning/environment:capability_extractor",
        "//modules/planning/validation:validation_supervisor",
        ":planning_runtime_context",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_cc_proto",
    ],
)

cc_test(
    name = "planning_cycle_result_test",
    srcs = ["planning_cycle_result_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        ":planning_cycle_result",
    ],
)

cc_test(
    name = "planning_runtime_status_builder_test",
    srcs = ["planning_runtime_status_builder_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        ":planning_runtime_status_builder",
    ],
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
        "@wheelos_common//execution_state:client",
        "@wheelos_common//execution_state:store",
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
        ":planning_runtime_context",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:local_view",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/environment:capability_extractor",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task_factory",
        "@core//cyber",
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "@wheelos_map//modules/map/hdmap",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
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
        "@wheelos_map//modules/map/hdmap:hdmap_util",
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
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:ego_info",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:published_trajectory_gear",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner:planner_selector",
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
    linkstatic = True,
    deps = [
        ":planning_shell_registry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "open_space_planning_test",
    size = "small",
    srcs = ["open_space_planning_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":open_space_planning",
        "@core//cyber/common:file",
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

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = if_gpu("planning-gpu", "planning"),
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libplanning_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = if_gpu("planning-gpu/lib", "planning/lib"),
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        ":planning_testdata_install",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_planning_data",
    srcs = [
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:decision_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_command_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:sl_boundary_py_pb2",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = if_gpu("planning-gpu", "planning"),
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_planning",
    srcs = [":pb_planning_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_planning_hdrs",
        ":package_planning_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_planning_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/planning/common:exported_source_files",
        "//modules/planning/common/path:exported_source_files",
        "//modules/planning/common/smoothers:exported_source_files",
        "//modules/planning/common/speed:exported_source_files",
        "//modules/planning/common/trajectory:exported_source_files",
        "//modules/planning/common/trajectory1d:exported_source_files",
        "//modules/planning/common/util:exported_source_files",
        "//modules/planning/constraint_checker:exported_source_files",
        "//modules/planning/environment:exported_source_files",
        "//modules/planning/math:exported_source_files",
        "//modules/planning/math/curve1d:exported_source_files",
        "//modules/planning/math/discretized_points_smoothing:exported_source_files",
        "//modules/planning/math/piecewise_jerk:exported_source_files",
        "//modules/planning/math/smoothing_spline:exported_source_files",
        "//modules/planning/mission_session:exported_source_files",
        "//modules/planning/mode:exported_source_files",
        "//modules/planning/open_space/coarse_trajectory_generator:exported_source_files",
        "//modules/planning/open_space/parking:exported_source_files",
        "//modules/planning/open_space/tools:exported_source_files",
        "//modules/planning/open_space/trajectory_smoother:exported_source_files",
        "//modules/planning/planner:exported_source_files",
        "//modules/planning/planner/public_road:exported_source_files",
        "//modules/planning/proto:exported_source_files",
        "//modules/planning/proto/math:exported_source_files",
        "//modules/planning/reference_line:exported_source_files",
        "//modules/planning/scenarios:exported_source_files",
        "//modules/planning/scenarios/common:exported_source_files",
        "//modules/planning/scenarios/cruise/lane_follow:exported_source_files",
        "//modules/planning/scenarios/cruise/lane_keeping:exported_source_files",
        "//modules/planning/scenarios/deciders:exported_source_files",
        "//modules/planning/scenarios/emergency/emergency_pull_over:exported_source_files",
        "//modules/planning/scenarios/emergency/emergency_stop:exported_source_files",
        "//modules/planning/scenarios/intersection/bare_intersection/unprotected:exported_source_files",
        "//modules/planning/scenarios/intersection/stop_sign/unprotected:exported_source_files",
        "//modules/planning/scenarios/intersection/traffic_light/protected:exported_source_files",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_left_turn:exported_source_files",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_right_turn:exported_source_files",
        "//modules/planning/scenarios/intersection/yield_sign:exported_source_files",
        "//modules/planning/scenarios/maneuver/deadend_turnaround:exported_source_files",
        "//modules/planning/scenarios/maneuver/escape:exported_source_files",
        "//modules/planning/scenarios/maneuver/narrow_street:exported_source_files",
        "//modules/planning/scenarios/maneuver/narrow_street_u_turn:exported_source_files",
        "//modules/planning/scenarios/park/mission_idle:exported_source_files",
        "//modules/planning/scenarios/park/park_and_go:exported_source_files",
        "//modules/planning/scenarios/park/pull_over:exported_source_files",
        "//modules/planning/scenarios/park/valet_parking:exported_source_files",
        "//modules/planning/scenarios/util:exported_source_files",
        "//modules/planning/tasks:exported_source_files",
        "//modules/planning/tasks/deciders:exported_source_files",
        "//modules/planning/tasks/deciders/creep_decider:exported_source_files",
        "//modules/planning/tasks/deciders/lane_change_decider:exported_source_files",
        "//modules/planning/tasks/deciders/open_space_decider:exported_source_files",
        "//modules/planning/tasks/deciders/path_assessment_decider:exported_source_files",
        "//modules/planning/tasks/deciders/path_bounds_decider:exported_source_files",
        "//modules/planning/tasks/deciders/path_decider:exported_source_files",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:exported_source_files",
        "//modules/planning/tasks/deciders/path_reuse_decider:exported_source_files",
        "//modules/planning/tasks/deciders/rss_decider:exported_source_files",
        "//modules/planning/tasks/deciders/rule_based_stop_decider:exported_source_files",
        "//modules/planning/tasks/deciders/speed_bounds_decider:exported_source_files",
        "//modules/planning/tasks/deciders/speed_decider:exported_source_files",
        "//modules/planning/tasks/deciders/st_bounds_decider:exported_source_files",
        "//modules/planning/tasks/deciders/utils:exported_source_files",
        "//modules/planning/tasks/optimizers:exported_source_files",
        "//modules/planning/tasks/optimizers/open_space_trajectory_generation:exported_source_files",
        "//modules/planning/tasks/optimizers/open_space_trajectory_partition:exported_source_files",
        "//modules/planning/tasks/optimizers/path_time_heuristic:exported_source_files",
        "//modules/planning/tasks/optimizers/piecewise_jerk_path:exported_source_files",
        "//modules/planning/tasks/optimizers/piecewise_jerk_speed:exported_source_files",
        "//modules/planning/tasks/utils:exported_source_files",
        "//modules/planning/tools:exported_source_files",
        "//modules/planning/traffic_rules:exported_source_files",
        "//modules/planning/validation:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = if_gpu("planning-gpu/src", "planning/src"),
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_filegroup(
    name = "package_planning_src",
    srcs = [":package_planning_src_data"],
)

pkg_files(
    name = "package_planning_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/planning/common:exported_source_headers",
        "//modules/planning/common/path:exported_source_headers",
        "//modules/planning/common/smoothers:exported_source_headers",
        "//modules/planning/common/speed:exported_source_headers",
        "//modules/planning/common/trajectory:exported_source_headers",
        "//modules/planning/common/trajectory1d:exported_source_headers",
        "//modules/planning/common/util:exported_source_headers",
        "//modules/planning/constraint_checker:exported_source_headers",
        "//modules/planning/environment:exported_source_headers",
        "//modules/planning/math:exported_source_headers",
        "//modules/planning/math/curve1d:exported_source_headers",
        "//modules/planning/math/discretized_points_smoothing:exported_source_headers",
        "//modules/planning/math/piecewise_jerk:exported_source_headers",
        "//modules/planning/math/smoothing_spline:exported_source_headers",
        "//modules/planning/mission_session:exported_source_headers",
        "//modules/planning/mode:exported_source_headers",
        "//modules/planning/open_space/coarse_trajectory_generator:exported_source_headers",
        "//modules/planning/open_space/parking:exported_source_headers",
        "//modules/planning/open_space/trajectory_smoother:exported_source_headers",
        "//modules/planning/planner:exported_source_headers",
        "//modules/planning/planner/public_road:exported_source_headers",
        "//modules/planning/reference_line:exported_source_headers",
        "//modules/planning/scenarios:exported_source_headers",
        "//modules/planning/scenarios/common:exported_source_headers",
        "//modules/planning/scenarios/cruise/lane_follow:exported_source_headers",
        "//modules/planning/scenarios/cruise/lane_keeping:exported_source_headers",
        "//modules/planning/scenarios/deciders:exported_source_headers",
        "//modules/planning/scenarios/emergency/emergency_pull_over:exported_source_headers",
        "//modules/planning/scenarios/emergency/emergency_stop:exported_source_headers",
        "//modules/planning/scenarios/intersection/bare_intersection/unprotected:exported_source_headers",
        "//modules/planning/scenarios/intersection/stop_sign/unprotected:exported_source_headers",
        "//modules/planning/scenarios/intersection/traffic_light/protected:exported_source_headers",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_left_turn:exported_source_headers",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_right_turn:exported_source_headers",
        "//modules/planning/scenarios/intersection/yield_sign:exported_source_headers",
        "//modules/planning/scenarios/maneuver/deadend_turnaround:exported_source_headers",
        "//modules/planning/scenarios/maneuver/escape:exported_source_headers",
        "//modules/planning/scenarios/maneuver/narrow_street:exported_source_headers",
        "//modules/planning/scenarios/maneuver/narrow_street_u_turn:exported_source_headers",
        "//modules/planning/scenarios/park/mission_idle:exported_source_headers",
        "//modules/planning/scenarios/park/park_and_go:exported_source_headers",
        "//modules/planning/scenarios/park/pull_over:exported_source_headers",
        "//modules/planning/scenarios/park/valet_parking:exported_source_headers",
        "//modules/planning/scenarios/util:exported_source_headers",
        "//modules/planning/tasks:exported_source_headers",
        "//modules/planning/tasks/deciders:exported_source_headers",
        "//modules/planning/tasks/deciders/creep_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/lane_change_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/open_space_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/path_assessment_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/path_bounds_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/path_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/path_reuse_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/rss_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/rule_based_stop_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/speed_bounds_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/speed_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/st_bounds_decider:exported_source_headers",
        "//modules/planning/tasks/deciders/utils:exported_source_headers",
        "//modules/planning/tasks/optimizers:exported_source_headers",
        "//modules/planning/tasks/optimizers/open_space_trajectory_generation:exported_source_headers",
        "//modules/planning/tasks/optimizers/open_space_trajectory_partition:exported_source_headers",
        "//modules/planning/tasks/optimizers/path_time_heuristic:exported_source_headers",
        "//modules/planning/tasks/optimizers/piecewise_jerk_path:exported_source_headers",
        "//modules/planning/tasks/optimizers/piecewise_jerk_speed:exported_source_headers",
        "//modules/planning/tasks/utils:exported_source_headers",
        "//modules/planning/traffic_rules:exported_source_headers",
        "//modules/planning/validation:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = if_gpu("planning-gpu/include", "planning/include"),
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_filegroup(
    name = "package_planning_hdrs",
    srcs = [":package_planning_hdrs_data"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = [
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:open_space_task_config_py_pb2",
        "//modules/planning/proto:planner_open_space_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:task_config_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_py_pb2",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "planning/include",
    strip_prefix = strip_prefix.from_root("modules/planning"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)
