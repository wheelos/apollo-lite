load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":routing_tester"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "routing/bin",
    strip_prefix = strip_prefix.from_root("modules/routing/tools/routing_tester"),
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

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        "//modules/routing:routing_service",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "*.pb.txt",
    ]),
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
