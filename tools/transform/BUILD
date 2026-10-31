load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("//tools/hardware:runtime.bzl", "runtime_layout", "runtime_package")

package(default_visibility = ["//visibility:public"])

runtime_layout(
    name = "runtime",
    components = ["@wheelos_transform//modules/transform:libstatic_transform_component.so"],
    runtime_data = ["@wheelos_transform//modules/transform:runtime_data"],
)

runtime_package(
    name = "package_files",
    runtime = ":runtime",
)

pkg_tar(
    name = "package",
    srcs = [":package_files"],
    extension = "tar.gz",
    package_file_name = "install.tar.gz",
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
