load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_binary(
    name = "libmonitor.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":monitor_lib",
    ],
)

cc_library(
    name = "monitor_lib",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    copts = MONITOR_COPTS,
    visibility = ["//visibility:private"],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/hardware:gps_monitor",
        "//modules/monitor/hardware:resource_monitor",
        "//modules/monitor/hardware:socket_can_monitor",
        "//modules/monitor/software:camera_monitor",
        "//modules/monitor/software:channel_monitor",
        "//modules/monitor/software:functional_safety_monitor",
        "//modules/monitor/software:latency_monitor",
        "//modules/monitor/software:localization_monitor",
        "//modules/monitor/software:module_monitor",
        "//modules/monitor/software:process_monitor",
        "//modules/monitor/software:recorder_monitor",
        "//modules/monitor/software:summary_monitor",
        "@core//cyber",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":monitor.BUILD",
        ":runtime_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "monitor",
    strip_prefix = strip_prefix.from_root("modules/monitor"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libmonitor.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "monitor/lib",
    strip_prefix = strip_prefix.from_root("modules/monitor"),
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

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_monitor_hdrs",
        ":package_monitor_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_monitor_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/monitor/common:exported_source_files",
        "//modules/monitor/hardware:exported_source_files",
        "//modules/monitor/software:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "monitor/src",
    strip_prefix = strip_prefix.from_root("modules/monitor"),
)

pkg_filegroup(
    name = "package_monitor_src",
    srcs = [":package_monitor_src_data"],
)

pkg_files(
    name = "package_monitor_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/monitor/common:exported_source_headers",
        "//modules/monitor/hardware:exported_source_headers",
        "//modules/monitor/software:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "monitor/include",
    strip_prefix = strip_prefix.from_root("modules/monitor"),
)

pkg_filegroup(
    name = "package_monitor_hdrs",
    srcs = [":package_monitor_hdrs_data"],
)
