load("//tools/products:defs.bzl", "runtime_product")

package(default_visibility = ["//visibility:public"])

runtime_product(
    name = "runtime",
    default_dag = "dag/transform/static_transform.dag",
    modules = ["@wheelos_transform//modules/transform:module"],
    package_file_name = "install.tar.gz",
)

alias(
    name = "run",
    actual = ":runtime_run",
)

alias(
    name = "package_files",
    actual = ":runtime_files",
)

alias(
    name = "package",
    actual = ":runtime_package",
)

alias(
    name = "install_dir",
    actual = ":runtime_install",
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
