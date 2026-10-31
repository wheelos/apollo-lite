load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_map_creator",
    srcs = ["ndt_map_creator.cc"],
    deps = [
        "//modules/ndt_localization/common/io:common_io",
        "//modules/ndt_localization/common/util",
        "//modules/ndt_localization/common/util:system_utility",
        "//modules/ndt_localization/map/base_map",
        "//modules/ndt_localization/map/ndt_map",
        "//modules/ndt_localization/map/ndt_map:ndt_map_pool",
        "@boost.filesystem",
        "@boost.program_options",
        "@boost.random",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@eigen",
    ],
)

pkg_files(
    name = "package_files_binaries",
    srcs = [":ndt_map_creator"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "ndt_localization/bin",
    strip_prefix = strip_prefix.from_root("modules/ndt_localization/map_creation"),
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
