load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

GUARDIAN_COPTS = ['-DMODULE_NAME=\\"guardian\\"']

cc_library(
    name = "guardian_component_lib",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = GUARDIAN_COPTS,
    deps = [
        "//modules/collision_guardian/proto:collision_guardian_cc_proto",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/guardian/proto:guardian_conf_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_cc_proto",
        "@wheelos_msgs//wheelos_msgs/guardian_msgs:guardian_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:system_status_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libguardian_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":guardian_component_lib",
    ],
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":guardian.BUILD",
        ":runtime_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "guardian",
    strip_prefix = strip_prefix.from_root("modules/guardian"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libguardian_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "guardian/lib",
    strip_prefix = strip_prefix.from_root("modules/guardian"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        ":pb_hdrs",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = ["//modules/guardian/proto:guardian_conf_cc_proto"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "guardian/include",
    strip_prefix = strip_prefix.from_root("modules/guardian"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_guardian_hdrs",
        ":package_guardian_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_guardian_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + ["//modules/guardian/proto:exported_source_files"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "guardian/src",
    strip_prefix = strip_prefix.from_root("modules/guardian"),
)

pkg_filegroup(
    name = "package_guardian_src",
    srcs = [":package_guardian_src_data"],
)

pkg_files(
    name = "package_guardian_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    attributes = pkg_attributes(mode = "0644"),
    prefix = "guardian/include",
    strip_prefix = strip_prefix.from_root("modules/guardian"),
)

pkg_filegroup(
    name = "package_guardian_hdrs",
    srcs = [":package_guardian_hdrs_data"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)
