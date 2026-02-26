load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library", "requires_cuda")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    library_dest = "map/lib",
    targets = [":pnc_map"],
    visibility = ["//visibility:public"],
)

cuda_library(
    name = "cuda_pnc_util",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    tags = ["exclude"],
    target_compatible_with = requires_cuda(),
    deps = [
        "//cyber",
        "//modules/common/math",
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
        "//modules/common/math",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "path",
    srcs = ["path.cc"],
    hdrs = ["path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "@com_google_absl//absl/algorithm",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_segments",
    srcs = ["route_segments.cc"],
    hdrs = ["route_segments.h"],
    copts = MAP_COPTS,
    deps = [
        ":path",
        "//modules/common/util",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map/hdmap",
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
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "@com_google_absl//absl/algorithm",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
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
        "//modules/common/util",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pnc_map_test",
    size = "small",
    srcs = ["pnc_map_test.cc"],
    data = [
        ":testdata",
    ],
    linkstatic = True,
    deps = [
        ":pnc_map",
        "//modules/common/util",
        "//modules/common/util:string_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "route_segments_test",
    size = "small",
    srcs = ["route_segments_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":route_segments",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
