load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("//tools/hardware:runtime.bzl", "runtime_layout")

package(default_visibility = ["//visibility:public"])

runtime_layout(
    name = "runtime",
    components = ["@wheelos_transform//modules/transform:libstatic_transform_component.so"],
    runtime_data = ["@wheelos_transform//modules/transform:runtime_data"],
)

pkg_tar(
    name = "install",
    srcs = [":runtime"],
    extension = "tar.gz",
)
