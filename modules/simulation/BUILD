load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

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
    name = "vehicle_model",
    srcs = ["vehicle/vehicle_model.cc"],
    hdrs = ["vehicle/vehicle_model.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":simulation_types",
    ],
)

cc_library(
    name = "ackermann_model",
    srcs = ["vehicle/ackermann_model.cc"],
    hdrs = ["vehicle/ackermann_model.h"],
    copts = SIMULATION_COPTS,
    deps = [":vehicle_model"],
)

cc_library(
    name = "four_wheel_steering_model",
    srcs = ["vehicle/four_wheel_steering_model.cc"],
    hdrs = ["vehicle/four_wheel_steering_model.h"],
    copts = SIMULATION_COPTS,
    deps = [":vehicle_model"],
)

cc_library(
    name = "simulation_engine",
    srcs = ["core/simulation_engine.cc"],
    hdrs = ["core/simulation_engine.h"],
    copts = SIMULATION_COPTS,
    deps = [
        ":ackermann_model",
        ":four_wheel_steering_model",
        ":simulation_gflags",
        ":simulation_types",
        ":simulator_backend",
        ":vehicle_model",
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
        "@wheelos_common//modules/common/adapters:adapter_gflags",
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
        ":vehicle_model",
        "@wheelos_common//modules/common/configs:config_gflags",
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
        ":four_wheel_steering_model",
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
    name = "cyber_adapter_test",
    size = "small",
    srcs = ["adapter/cyber_adapter_test.cc"],
    copts = SIMULATION_COPTS,
    deps = [
        ":cyber_adapter",
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/vehicle_state:vehicle_state_provider",
        "@com_google_googletest//:gtest_main",
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

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*",
        "dag/*.dag",
        "model/*",
    ]),
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_data"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "simulation",
    strip_prefix = strip_prefix.from_root("modules/simulation"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libsimulation_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "simulation/lib",
    strip_prefix = strip_prefix.from_root("modules/simulation"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
