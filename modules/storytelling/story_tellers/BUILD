load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_teller",
    hdrs = ["base_teller.h"],
    deps = [
        "@wheelos_msgs//wheelos_msgs/storytelling_msgs:story_cc_proto",
    ],
)

cc_library(
    name = "close_to_junction_teller",
    srcs = ["close_to_junction_teller.cc"],
    hdrs = ["close_to_junction_teller.h"],
    deps = [
        ":base_teller",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/storytelling:frame_manager",
        "//modules/storytelling/common:storytelling_gflags",
        "//modules/storytelling/proto:storytelling_config_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
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
