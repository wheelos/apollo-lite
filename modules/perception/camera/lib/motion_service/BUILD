load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_libraries",
    srcs = [":libmotion_service.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "perception/lib/camera/lib/motion_service",
    strip_prefix = strip_prefix.from_root("modules/perception/camera/lib/motion_service"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_libraries"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

cc_library(
    name = "motion_service_lib",
    srcs = ["motion_service.cc"],
    hdrs = ["motion_service.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/camera/lib/motion",
        "//modules/perception/lib/registerer",
        "//modules/perception/onboard/proto:motion_service_cc_proto",
        "//modules/perception/proto:motion_service_cc_proto",
        "@boost.algorithm",
        "@boost.format",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_binary(
    name = "libmotion_service.so",
    linkshared = True,
    linkstatic = False,
    deps = [":motion_service_lib"],
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
