load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

proto_library(
    name = "bevfusion_proto",
    srcs = ["proto/bevfusion.proto"],
)

cc_proto_library(
    name = "bevfusion_cc_proto",
    deps = [":bevfusion_proto"],
)

cc_library(
    name = "bevfusion_input",
    srcs = [
        "types/bevfusion_input.cc",
        "types/bounded_synchronizer.cc",
    ],
    hdrs = [
        "types/bevfusion_input.h",
        "types/bounded_synchronizer.h",
    ],
)

cc_test(
    name = "bevfusion_input_test",
    size = "small",
    srcs = ["types/bevfusion_input_test.cc"],
    deps = [
        ":bevfusion_input",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "bounded_synchronizer_test",
    size = "small",
    srcs = ["types/bounded_synchronizer_test.cc"],
    deps = [
        ":bevfusion_input",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "bevfusion_runtime_backend_test",
    size = "small",
    srcs = ["inference/bevfusion_runtime_backend_test.cc"],
    deps = [
        ":bevfusion_runner",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "bevfusion_runtime_abi_test",
    size = "small",
    srcs = ["inference/bevfusion_runtime_abi_test.cc"],
    data = [":libbevfusion_runtime.so"],
    env = {
        "BEVFUSION_RUNTIME_DSO": "$(location :libbevfusion_runtime.so)",
    },
    linkopts = ["-ldl"],
    deps = [
        ":bevfusion_runner",
        "@com_google_googletest//:gtest_main",
    ],
)

cuda_library(
    name = "bevfusion_cuda_kernels",
    srcs = [
        "third_party/bevfusion/bevfusion/camera-backbone.cu",
        "third_party/bevfusion/bevfusion/camera-bevpool.cu",
        "third_party/bevfusion/bevfusion/camera-depth.cu",
        "third_party/bevfusion/bevfusion/camera-geometry.cu",
        "third_party/bevfusion/bevfusion/camera-normalization.cu",
        "third_party/bevfusion/bevfusion/camera-vtransform.cu",
        "third_party/bevfusion/bevfusion/head-transbbox.cu",
        "third_party/bevfusion/bevfusion/lidar-voxelization.cu",
        "third_party/bevfusion/bevfusion/transfusion.cu",
        "third_party/bevfusion/common/tensor.cu",
    ],
    hdrs = glob([
        "third_party/bevfusion/bevfusion/*.hpp",
        "third_party/bevfusion/common/*.hpp",
        "third_party/bevfusion/common/*.cuh",
    ]),
    copts = ["-std=c++17"],
    includes = ["third_party/bevfusion"],
    deps = [
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "bevfusion_upstream_core",
    srcs = [
        "third_party/bevfusion/bevfusion/bevfusion.cpp",
        "third_party/bevfusion/bevfusion/lidar-scn.cpp",
        "third_party/bevfusion/common/tensorrt.cpp",
    ],
    hdrs = glob([
        "third_party/bevfusion/bevfusion/*.hpp",
        "third_party/bevfusion/common/*.hpp",
    ]),
    copts = ["-std=c++17"],
    includes = ["third_party/bevfusion"],
    deps = [
        ":bevfusion_cuda_kernels",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
        "@wheelos_common//perception/model_io/onnx:sparse_model",
        "@wheelos_common//perception/ops/spconv:spconv_backbone",
    ],
    alwayslink = True,
)

cc_library(
    name = "bevfusion_runtime_api",
    srcs = ["inference/bevfusion_runtime_api.cc"],
    hdrs = ["inference/bevfusion_runtime_api.h"],
    copts = ["-std=c++17"],
    deps = [
        ":bevfusion_input",
        ":bevfusion_upstream_core",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libbevfusion_runtime.so",
    linkopts = ["-Wl,-rpath,$$ORIGIN"],
    linkshared = True,
    linkstatic = True,
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [":bevfusion_runtime_api"],
)

cc_library(
    name = "bevfusion_runner",
    srcs = [
        "inference/bevfusion_runner.cc",
        "inference/bevfusion_runtime_backend.cc",
    ],
    hdrs = [
        "inference/bevfusion_runner.h",
        "inference/bevfusion_runtime_api.h",
        "inference/bevfusion_runtime_backend.h",
    ],
    linkopts = ["-ldl"],
    deps = [
        ":bevfusion_cc_proto",
        ":bevfusion_input",
    ],
)

cc_binary(
    name = "bevfusion_offline",
    srcs = ["tools/bevfusion_offline.cc"],
    data = [":libbevfusion_runtime.so"],
    deps = [
        ":bevfusion_runner",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "bevfusion_component_lib",
    srcs = ["component/bevfusion_component.cc"],
    hdrs = ["component/bevfusion_component.h"],
    copts = ['-DMODULE_NAME=\\"bevfusion\\"'],
    deps = [
        ":bevfusion_cc_proto",
        ":bevfusion_runner",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libbevfusion_component_plugin.so",
    linkshared = True,
    linkstatic = False,
    deps = [":bevfusion_component_lib"],
)

filegroup(
    name = "runtime_config",
    srcs = glob([
        "conf/**",
        "dag/**",
    ]),
)

pkg_files(
    name = "package_bevfusion_spconv_licenses_data",
    srcs = ["@spconv//:licenses"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "bevfusion/share/licenses/spconv",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "package_bevfusion_spconv_licenses",
    srcs = [":package_bevfusion_spconv_licenses_data"],
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_config"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "bevfusion",
    strip_prefix = strip_prefix.from_root("modules/bevfusion"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [
        ":libbevfusion_component_plugin.so",
        ":libbevfusion_runtime.so",
    ],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "bevfusion/lib",
    strip_prefix = strip_prefix.from_root("modules/bevfusion"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_bevfusion_spconv_licenses",
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
