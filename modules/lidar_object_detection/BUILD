load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointpillars_runtime_api",
    srcs = ["inference/pointpillars_runtime_api.cc"],
    hdrs = ["inference/pointpillars_runtime_api.h"],
    deps = [
        "//modules/lidar_object_detection/third_party/pointpillars",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libpointpillars_runtime.so",
    linkshared = True,
    linkstatic = True,
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [":pointpillars_runtime_api"],
)

cc_library(
    name = "pointpillars_input",
    srcs = ["types/pointpillars_input.cc"],
    hdrs = ["types/pointpillars_input.h"],
    deps = ["@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto"],
)

cc_test(
    name = "pointpillars_input_test",
    size = "small",
    srcs = ["types/pointpillars_input_test.cc"],
    deps = [
        ":pointpillars_input",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "model_input",
    srcs = ["types/model_input.cc"],
    hdrs = ["types/model_input.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "sweep_fusion",
    srcs = ["types/sweep_fusion.cc"],
    hdrs = ["types/sweep_fusion.h"],
    deps = [
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runner",
    srcs = ["inference/centerpoint_runner.cc"],
    hdrs = [
        "inference/centerpoint_backend.h",
        "inference/centerpoint_runner.h",
    ],
    deps = [
        ":model_input",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runtime_backend",
    srcs = ["inference/centerpoint_runtime_backend.cc"],
    hdrs = [
        "inference/centerpoint_runtime_api.h",
        "inference/centerpoint_runtime_backend.h",
    ],
    linkopts = ["-ldl"],
    deps = [
        ":centerpoint_runner",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runtime_api",
    srcs = ["inference/centerpoint_runtime_api.cc"],
    hdrs = ["inference/centerpoint_runtime_api.h"],
    deps = [
        "//modules/lidar_object_detection/third_party/centerpoint",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_object_detection_component_lib",
    srcs = ["component/lidar_object_detection_component.cc"],
    hdrs = [
        "component/lidar_object_detection_component.h",
        "inference/pointpillars_runtime_api.h",
    ],
    copts = ['-DMODULE_NAME=\\"lidar_object_detection\\"'],
    linkopts = ["-ldl"],
    deps = [
        ":centerpoint_runner",
        ":centerpoint_runtime_backend",
        ":pointpillars_input",
        ":sweep_fusion",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "liblidar_object_detection_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_object_detection_component_lib"],
)

cc_test(
    name = "lidar_object_detection_component_test",
    size = "small",
    srcs = ["component/lidar_object_detection_component_test.cc"],
    tags = ["manual"],
    deps = [
        ":lidar_object_detection_component_lib",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
    ],
)

cc_binary(
    name = "libcenterpoint_runtime.so",
    linkopts = [
        "-Wl,-rpath,$$ORIGIN",
        "-Wl,-z,defs",
    ],
    linkshared = True,
    linkstatic = True,
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [":centerpoint_runtime_api"],
)

cc_library(
    name = "offline_result_writer",
    srcs = ["tools/offline_result_writer.cc"],
    hdrs = ["tools/offline_result_writer.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
    ],
)

cc_binary(
    name = "pcd_to_centerpoint_input",
    srcs = ["tools/pcd_to_centerpoint_input.cc"],
    deps = [
        ":model_input",
        "//modules/perception/lidar/common:pcl_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "model_input_test",
    size = "small",
    srcs = ["types/model_input_test.cc"],
    deps = [
        ":model_input",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "sweep_fusion_test",
    size = "small",
    srcs = ["types/sweep_fusion_test.cc"],
    deps = [
        ":model_input",
        ":sweep_fusion",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

filegroup(
    name = "runtime_config",
    srcs = [
        "conf/lidar_object_detection_component.pb.txt",
        "dag/lidar_object_detection.dag",
    ],
)

pkg_files(
    name = "package_centerpoint_spconv_licenses_data",
    srcs = ["@spconv//:licenses"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lidar_object_detection/share/licenses/spconv",
    strip_prefix = strip_prefix.files_only(),
)

pkg_filegroup(
    name = "package_centerpoint_spconv_licenses",
    srcs = [":package_centerpoint_spconv_licenses_data"],
)

pkg_files(
    name = "package_component_data",
    srcs = [":runtime_config"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lidar_object_detection",
    strip_prefix = strip_prefix.from_root("modules/lidar_object_detection"),
)

pkg_files(
    name = "package_component_libraries",
    srcs = [":liblidar_object_detection_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "lidar_object_detection/lib",
    strip_prefix = strip_prefix.from_root("modules/lidar_object_detection"),
)

pkg_filegroup(
    name = "package_component",
    srcs = [
        ":package_component_data",
        ":package_component_libraries",
    ],
)

pkg_files(
    name = "package_centerpoint_libraries",
    srcs = [":libcenterpoint_runtime.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "lidar_object_detection/lib",
    strip_prefix = strip_prefix.from_root("modules/lidar_object_detection"),
)

pkg_filegroup(
    name = "package_centerpoint",
    srcs = [
        ":package_centerpoint_libraries",
        ":package_centerpoint_spconv_licenses",
        ":package_component",
    ],
)

pkg_files(
    name = "package_pointpillars_data",
    srcs = ["tools/run_pointpillars.py"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lidar_object_detection",
    strip_prefix = strip_prefix.from_root("modules/lidar_object_detection"),
)

pkg_files(
    name = "package_pointpillars_libraries",
    srcs = [":libpointpillars_runtime.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "lidar_object_detection/lib",
    strip_prefix = strip_prefix.from_root("modules/lidar_object_detection"),
)

pkg_filegroup(
    name = "package_pointpillars",
    srcs = [
        ":package_component",
        ":package_pointpillars_data",
        ":package_pointpillars_libraries",
    ],
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_centerpoint",
        ":package_pointpillars",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_install(
    name = "centerpoint_dir",
    srcs = [":package_centerpoint"],
    tags = ["manual"],
)

pkg_install(
    name = "pointpillars_dir",
    srcs = [":package_pointpillars"],
    tags = ["manual"],
)
