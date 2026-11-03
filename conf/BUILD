load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "vehicle_config_data",
    srcs = ["vehicles/default/vehicle_param.pb.txt"],
)

filegroup(
    name = "global_flagfile",
    srcs = ["global_flagfile.txt"],
)

pkg_files(
    name = "package_files",
    srcs = [
        ":global_flagfile",
        ":vehicle_config_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "conf",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
