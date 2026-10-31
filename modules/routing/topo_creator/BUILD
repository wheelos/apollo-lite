load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":topo_creator"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "modules/routing/topo_creator",
    strip_prefix = strip_prefix.from_root("modules/routing/topo_creator"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

cc_library(
    name = "edge_creator",
    srcs = ["edge_creator.cc"],
    hdrs = ["edge_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_library(
    name = "graph_creator",
    srcs = ["graph_creator.cc"],
    hdrs = ["graph_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":edge_creator",
        ":node_creator",
        "@com_google_absl//absl/strings",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_cc_proto",
    ],
)

cc_test(
    name = "graph_creator_test",
    size = "small",
    srcs = ["graph_creator_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":graph_creator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "node_creator",
    srcs = ["node_creator.cc"],
    hdrs = ["node_creator.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_lane_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator.cc"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":graph_creator",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
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
