load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "routing_service",
    srcs = ["routing.cc"],
    hdrs = ["routing.h"],
    copts = ROUTING_COPTS,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/util",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
    ]),
)

install(
    name = "install",
    data = [
        ":routing.BUILD",
        ":runtime_data",
    ],
    data_dest = "routing",
    library_dest = "routing/lib",
    targets = [
        ":routing_service",
    ],
    deps = [
        ":pb_hdrs",
        ":pb_routing",
        "//modules/routing/tools:install",
        "//modules/routing/tools/routing_tester:install",
        "//modules/routing/topo_creator:install",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
    ],
    data_dest = "routing/include",
)

install_files(
    name = "pb_routing",
    dest = "routing",
    files = [
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_routing_hdrs",
        ":install_routing_src",
    ],
)

install_src_files(
    name = "install_routing_src",
    dest = "routing/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_routing_hdrs",
    dest = "routing/include",
    filter = "*.h",
    src_dir = ["."],
)

cpplint()
