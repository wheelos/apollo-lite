load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":records_to_offline_data"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "prediction/bin",
    strip_prefix = strip_prefix.from_root("modules/prediction/pipeline"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        "//modules/prediction/common:message_process",
        "@boost.filesystem",
        "@com_google_absl//absl/strings",
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
