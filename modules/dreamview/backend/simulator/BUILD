load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_base",
    hdrs = [
        "base/entity.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "simulator_scenario",
    srcs = [
        "scenario/parser.cc",
        "scenario/manager.cc",
    ],
    hdrs = [
        "scenario/scenario.h",
        "scenario/parser.h",
        "scenario/manager.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_base",
        ":simulator_entity_ego",
        ":simulator_entity_env",
        "//modules/dreamview/backend/map:map_service",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "simulator_entity_ego",
    srcs = [
        "entity/ego/ego_model.cc",
    ],
    hdrs = [
        "entity/ego/ego_model_base.h",
        "entity/ego/ego_model.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_base",
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_pb",
        "//modules/common_msgs/chassis_msgs:chassis_cc_pb",
        "//modules/common_msgs/localization_msgs:localization_cc_pb",
        "//modules/dreamview/backend/map:map_service",
    ],
)

cc_library(
    name = "simulator_entity_env",
    srcs = [
        "entity/env/obstacle_manager.cc",
        "entity/env/traffic_light_manager.cc",
    ],
    hdrs = [
        "entity/env/obstacle_manager.h",
        "entity/env/traffic_light_manager.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_base",
        "//cyber",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_pb",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_pb",
        "//modules/dreamview/backend/map:map_service",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "simulator_core",
    srcs = [
        "core/engine.cc",
    ],
    hdrs = [
        "core/engine.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_base",
        ":simulator_scenario",
        ":simulator_entity_ego",
        ":simulator_entity_env",
        "//cyber",
        "//modules/dreamview/backend/map:map_service",
    ],
)

cc_library(
    name = "simulator_evaluation",
    srcs = [
        "evaluation/evaluator.cc",
        "evaluation/metrics/max_sim_time_metric.cc",
    ],
    hdrs = [
        "evaluation/evaluation_manager.h",
        "evaluation/evaluator.h",
        "evaluation/evaluator_base.h",
        "evaluation/metrics/metric_base.h",
        "evaluation/metrics/max_sim_time_metric.h",
    ],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_base",
    ],
)

cc_library(
    name = "simulation_manager",
    srcs = ["simulation_manager.cc"],
    hdrs = ["simulation_manager.h"],
    copts = ["-DMODULE_NAME=\\"dreamview\\""],
    deps = [
        ":simulator_core",
        ":simulator_evaluation",
        ":simulator_scenario",
        "//modules/dreamview/backend/map:map_service",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_binary(
    name = "simulator_main",
    srcs = ["simulator_main.cc"],
    copts = ["-DMODULE_NAME=\"dreamview\""],
    deps = [
        ":simulation_manager",
        "//cyber",
        "//modules/dreamview/backend/map:map_service",
        "@com_github_gflags_gflags//:gflags",
    ],
)
