load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library", "requires_cuda")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

pkg_files(
    name = "package_files_archives",
    srcs = [":pnc_map"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/map/pnc_map"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cuda_library(
    name = "cuda_pnc_util",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    tags = ["exclude"],
    target_compatible_with = requires_cuda(),
    deps = [
        "@wheelos_common//modules/common/math",
        "@core//cyber/common:log",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
    ],
)

cc_test(
    name = "cuda_util_test",
    size = "small",
    srcs = [
        "cuda_util_test.cc",
    ],
    linkstatic = True,
    tags = ["exclude"],
    deps = [
        ":cuda_pnc_util",
        "@wheelos_common//modules/common/math",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "path",
    srcs = ["path.cc"],
    hdrs = ["path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@com_google_absl//absl/algorithm",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@core//cyber/common:log",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_map//modules/map/hdmap",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_lane_cc_proto",
    ],
)

cc_library(
    name = "route_segments",
    srcs = ["route_segments.cc"],
    hdrs = ["route_segments.h"],
    copts = MAP_COPTS,
    deps = [
        ":path",
        "@wheelos_common//modules/common/util",
        "@wheelos_common//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_map//modules/map/hdmap",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_library(
    name = "pnc_map",
    srcs = [
        "pnc_map.cc",
    ],
    hdrs = [
        "path.h",
        "pnc_map.h",
        "route_segments.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":path",
        ":route_segments",
        "@com_google_absl//absl/algorithm",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@core//cyber/common:log",
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/util",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_common//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@wheelos_map//modules/map/hdmap",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_lane_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_test(
    name = "pnc_path_test",
    size = "small",
    srcs = ["path_test.cc"],
    deps = [
        ":path",
        "@wheelos_common//modules/common/util",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
