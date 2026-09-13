load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

SIMULATION_COPTS = ['-DMODULE_NAME=\\"simulation\\"']

cc_library(
    name = "simulation_gflags",
    srcs = ["common/simulation_gflags.cc"],
    hdrs = ["common/simulation_gflags.h"],
    copts = SIMULATION_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "simulation_types",
    hdrs = ["core/simulation_types.h"],
)

cc_library(
    name = "simulator_backend",
    srcs = [
        "backend/kinematic_backend.cc",
        "backend/mujoco_backend.cc",
    ],
    hdrs = [
        "backend/kinematic_backend.h",
        "backend/mujoco_backend.h",
        "backend/simulator_backend.h",
    ],
    copts = SIMULATION_COPTS + ["-DUSE_MUJOCO"],
    deps = [
        ":simulation_gflags",
        ":simulation_types",
        "@core//cyber",
        "@mujoco",
    ],
)

cc_library(
    name = "ackermann_model",
    srcs = ["vehicle/ackermann_model.cc"],
    hdrs = ["vehicle/ackermann_model.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":simulation_types",
    ],
)

cc_library(
    name = "simulation_engine",
    srcs = ["core/simulation_engine.cc"],
    hdrs = ["core/simulation_engine.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":ackermann_model",
        ":simulation_gflags",
        ":simulation_types",
        ":simulator_backend",
        "@core//cyber",
    ],
)

cc_library(
    name = "cyber_adapter",
    srcs = ["adapter/cyber_adapter.cc"],
    hdrs = ["adapter/cyber_adapter.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":simulation_gflags",
        ":simulation_types",
        "//modules/common/adapters:adapter_gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_library(
    name = "simulation_component_lib",
    srcs = ["simulation_component.cc"],
    hdrs = ["simulation_component.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":cyber_adapter",
        ":simulation_engine",
        ":simulation_gflags",
        "//modules/common/configs:config_gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libsimulation_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":simulation_component_lib",
        ":simulation_gflags",
    ],
)

cc_test(
    name = "ackermann_model_test",
    size = "small",
    srcs = ["vehicle/ackermann_model_test.cc"],
    copts = SIMULATION_COPTS,
    deps = [
        ":ackermann_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mujoco_backend_test",
    size = "small",
    srcs = ["backend/mujoco_backend_test.cc"],
    copts = SIMULATION_COPTS + ["-DUSE_MUJOCO"],
    data = [":runtime_data"],
    deps = [
        ":simulator_backend",
        "@com_google_googletest//:gtest_main",
        "@mujoco",
    ],
)

cc_test(
    name = "simulation_engine_test",
    size = "small",
    srcs = ["core/simulation_engine_test.cc"],
    copts = SIMULATION_COPTS,
    data = [":runtime_data"],
    deps = [
        ":simulation_engine",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "simulation_control_test",
    size = "small",
    srcs = ["simulation_control_test.cc"],
    copts = SIMULATION_COPTS,
    data = [
        ":runtime_data",
        "//modules/control:test_data",
    ],
    deps = [
        ":cyber_adapter",
        ":simulation_engine",
        "//modules/common/configs:config_gflags",
        "//modules/control/common:control_gflags",
        "//modules/control/common:dependency_injector",
        "//modules/control/controller:controller_agent",
        "//modules/control/proto:control_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
        "dag/*.dag",
        "model/*",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "simulation",
    library_dest = "simulation/lib",
    targets = [
        ":libsimulation_component.so",
    ],
)
