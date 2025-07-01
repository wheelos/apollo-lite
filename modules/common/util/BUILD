load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "util_testdata",
    data = [
        ":util_test_data",
    ],
    data_dest = "common/addition_data/util",
)

filegroup(
    name = "util_test_data",
    srcs = glob([
        "testdata/*",
    ]),
)

cc_library(
    name = "util_tool",
    hdrs = glob(
        ["*.h"],
        exclude = [
            "point_factory.h",
            "points_downsampler.h",
            "util.h",
        ],
    ),
    visibility = ["//visibility:public"],
    deps = [
        ":color",
        ":data_extraction",
        ":eigen_defs",
        ":factory",
        ":future",
        ":json_util",
        ":lru_cache",
        ":map_util",
        ":message_util",
        ":perf_util",
        ":string_util",
        ":time_conversion",
        "//cyber",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
        "@eigen",
    ],
)

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ]
)

cc_library(
    name = "future",
    hdrs = ["future.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ]
)

cc_library(
    name = "point_factory",
    hdrs = ["point_factory.h"],
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ]
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = ["util_test.cc"],
    deps = [
        ":util_lib",
        "//modules/common/util/proto:simple_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "lru_cache",
    hdrs = ["lru_cache.h"]
)

cc_library(
    name = "color",
    hdrs = ["color.h"]
)

cc_library(
    name = "data_extraction",
    srcs = ["data_extraction.cc"],
    hdrs = ["data_extraction.h"],
    deps = [
        "//cyber",
        "@boost.filesystem",
        "@boost.range",
    ]
)

cc_library(
    name = "string_util",
    srcs = ["string_util.cc"],
    hdrs = ["string_util.h"],
    deps = [
        ":future",
        "@com_google_absl//absl/algorithm",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ]
)

cc_library(
    name = "message_util",
    hdrs = ["message_util.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ]
)

cc_test(
    name = "message_util_test",
    size = "small",
    srcs = ["message_util_test.cc"],
    tags = [
        "external",
    ],
    deps = [
        ":message_util",
        "//modules/common/util/proto:simple_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "string_util_test",
    size = "small",
    srcs = ["string_util_test.cc"],
    deps = [
        ":string_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "time_conversion",
    hdrs = ["time_conversion.h"]
)

cc_library(
    name = "map_util",
    hdrs = ["map_util.h"],
    deps = [
        "//cyber",
        "@com_google_protobuf//:protobuf",
    ]
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    deps = [
        "//cyber",
    ]
)

cc_test(
    name = "factory_test",
    size = "small",
    srcs = ["factory_test.cc"],
    deps = [
        ":factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lru_cache_test",
    size = "small",
    srcs = ["lru_cache_test.cc"],
    deps = [
        ":lru_cache",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "point_factory.h",
        "points_downsampler.h",
        "util.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":point_factory",
        ":points_downsampler",
        ":util_lib",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "points_downsampler",
    hdrs = ["points_downsampler.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
    ]
)

cc_test(
    name = "points_downsampler_test",
    size = "small",
    srcs = ["points_downsampler_test.cc"],
    deps = [
        ":points_downsampler",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "json_util",
    srcs = ["json_util.cc"],
    hdrs = ["json_util.h"],
    deps = [
        "//cyber",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
    ]
)

cc_test(
    name = "json_util_test",
    size = "small",
    srcs = ["json_util_test.cc"],
    deps = [
        ":json_util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "eigen_defs",
    hdrs = ["eigen_defs.h"],
    deps = ["@eigen"]
)

cc_library(
    name = "perf_util",
    srcs = ["perf_util.cc"],
    hdrs = ["perf_util.h"],
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
    ]
)

cc_test(
    name = "perf_util_test",
    size = "small",
    srcs = ["perf_util_test.cc"],
    deps = [
        ":perf_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
