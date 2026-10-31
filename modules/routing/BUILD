load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

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
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/util",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/routing/proto:routing_config_cc_proto",
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

pkg_files(
    name = "package_files_data",
    srcs = [
        ":routing.BUILD",
        ":runtime_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "routing",
    strip_prefix = strip_prefix.from_root("modules/routing"),
)

pkg_files(
    name = "package_files_archives",
    srcs = [":routing_service"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/routing"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_archives",
        ":package_files_data",
        ":pb_hdrs",
        ":pb_routing",
        "//modules/routing/tools:package_files",
        "//modules/routing/tools/routing_tester:package_files",
        "//modules/routing/topo_creator:package_files",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = [
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "routing/include",
    strip_prefix = strip_prefix.from_root("modules/routing"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_files(
    name = "pb_routing_data",
    srcs = ["@wheelos_msgs//wheelos_msgs/routing_msgs:routing_py_pb2"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "routing",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_routing",
    srcs = [":pb_routing_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_routing_hdrs",
        ":package_routing_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_routing_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/routing/common:exported_source_files",
        "//modules/routing/core:exported_source_files",
        "//modules/routing/graph:exported_source_files",
        "//modules/routing/proto:exported_source_files",
        "//modules/routing/strategy:exported_source_files",
        "//modules/routing/tools:exported_source_files",
        "//modules/routing/tools/routing_tester:exported_source_files",
        "//modules/routing/topo_creator:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "routing/src",
    strip_prefix = strip_prefix.from_root("modules/routing"),
)

pkg_filegroup(
    name = "package_routing_src",
    srcs = [":package_routing_src_data"],
)

pkg_files(
    name = "package_routing_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/routing/common:exported_source_headers",
        "//modules/routing/core:exported_source_headers",
        "//modules/routing/graph:exported_source_headers",
        "//modules/routing/strategy:exported_source_headers",
        "//modules/routing/topo_creator:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "routing/include",
    strip_prefix = strip_prefix.from_root("modules/routing"),
)

pkg_filegroup(
    name = "package_routing_hdrs",
    srcs = [":package_routing_hdrs_data"],
)
