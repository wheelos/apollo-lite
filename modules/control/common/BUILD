load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = ["control_gflags.h"],
    copts = CONTROL_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "hysteresis_filter",
    srcs = [
        "hysteresis_filter.cc",
    ],
    hdrs = ["hysteresis_filter.h"],
    copts = CONTROL_COPTS,
)

cc_library(
    name = "interpolation_1d",
    srcs = ["interpolation_1d.cc"],
    hdrs = ["interpolation_1d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = ["interpolation_2d.cc"],
    hdrs = ["interpolation_2d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "leadlag_controller",
    srcs = ["leadlag_controller.cc"],
    hdrs = ["leadlag_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/proto:leadlag_conf_cc_proto",
    ],
)

cc_library(
    name = "mrac_controller",
    srcs = ["mrac_controller.cc"],
    hdrs = ["mrac_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/proto:mrac_conf_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_BC_controller",
    srcs = ["pid_BC_controller.cc"],
    hdrs = ["pid_BC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_IC_controller",
    srcs = ["pid_IC_controller.cc"],
    hdrs = ["pid_IC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = ["trajectory_analyzer.cc"],
    hdrs = ["trajectory_analyzer.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_gflags",
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "terminal_control_helper",
    srcs = ["terminal_control_helper.cc"],
    hdrs = ["terminal_control_helper.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "control_command_goal",
    srcs = ["control_command_goal.cc"],
    hdrs = ["control_command_goal.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "motion_execution_validator",
    srcs = ["motion_execution_validator.cc"],
    hdrs = ["motion_execution_validator.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common_msgs/planning_msgs:motion_execution_cc_proto",
    ],
)

cc_library(
    name = "executor_arbiter",
    srcs = ["executor_arbiter.cc"],
    hdrs = ["executor_arbiter.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common_msgs/planning_msgs:motion_execution_cc_proto",
    ],
)

cc_library(
    name = "motion_command_adapter",
    srcs = ["motion_command_adapter.cc"],
    hdrs = ["motion_command_adapter.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common_msgs/planning_msgs:motion_execution_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "motion_execution_manager",
    srcs = ["motion_execution_manager.cc"],
    hdrs = ["motion_execution_manager.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":motion_execution_validator",
        "//modules/common_msgs/planning_msgs:motion_execution_cc_proto",
    ],
)

cc_library(
    name = "strategy_orchestrator",
    srcs = ["strategy_orchestrator.cc"],
    hdrs = ["strategy_orchestrator.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_command_goal",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "common",
    copts = CONTROL_COPTS,
    deps = [
        ":control_command_goal",
        ":control_gflags",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":executor_arbiter",
        ":motion_command_adapter",
        ":motion_execution_manager",
        ":motion_execution_validator",
        ":pid_controller",
        ":strategy_orchestrator",
        ":terminal_control_helper",
        ":trajectory_analyzer",
    ],
)

cc_test(
    name = "executor_arbiter_test",
    size = "small",
    srcs = ["executor_arbiter_test.cc"],
    deps = [
        ":executor_arbiter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_command_adapter_test",
    size = "small",
    srcs = ["motion_command_adapter_test.cc"],
    deps = [
        ":motion_command_adapter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_chain_contract_test",
    size = "small",
    srcs = ["motion_chain_contract_test.cc"],
    linkopts = ["-fopenmp"],
    deps = [
        ":executor_arbiter",
        ":motion_execution_manager",
        "//modules/planning/common:gtest_exit_main",
        "//modules/planning/common:motion_plan_builder",
    ],
)

cc_test(
    name = "hysteresis_filter_test",
    size = "small",
    srcs = ["hysteresis_filter_test.cc"],
    deps = [
        ":hysteresis_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "interpolation_1d_test",
    size = "small",
    srcs = ["interpolation_1d_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":interpolation_1d",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "interpolation_2d_test",
    size = "small",
    srcs = ["interpolation_2d_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":interpolation_2d",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "leadlag_controller_test",
    size = "small",
    srcs = ["leadlag_controller_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":leadlag_controller",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:leadlag_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mrac_controller_test",
    size = "small",
    srcs = ["mrac_controller_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":mrac_controller",
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:mrac_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["pid_controller_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_BC_controller_test",
    size = "small",
    srcs = ["pid_BC_controller_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":pid_BC_controller",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_IC_controller_test",
    size = "small",
    srcs = ["pid_IC_controller_test.cc"],
    data = ["//modules/control:test_data"],
    deps = [
        ":pid_IC_controller",
        "//cyber",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "terminal_control_helper_test",
    size = "small",
    srcs = ["terminal_control_helper_test.cc"],
    deps = [
        ":terminal_control_helper",
        "//modules/planning/common:gtest_exit_main",
    ],
)

cc_test(
    name = "strategy_orchestrator_test",
    size = "small",
    srcs = ["strategy_orchestrator_test.cc"],
    deps = [
        ":strategy_orchestrator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_execution_validator_test",
    size = "small",
    srcs = ["motion_execution_validator_test.cc"],
    deps = [
        ":motion_execution_validator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_execution_manager_test",
    size = "small",
    srcs = ["motion_execution_manager_test.cc"],
    deps = [
        ":motion_execution_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_analyzer_test",
    size = "small",
    srcs = ["trajectory_analyzer_test.cc"],
    deps = [
        ":trajectory_analyzer",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    deps = [
        "//modules/common/vehicle_state:vehicle_state_provider",
    ],
)

install(
    name = "install",
    library_dest = "control/lib",
    targets = [
        ":hysteresis_filter",
    ],
)

cpplint()
