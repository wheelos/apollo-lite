load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(
    default_visibility = ["//visibility:public"],
)

pkg_files(
    name = "package_scripts_data",
    srcs = [
        ":apollo_base",
        ":bootstrap",
        ":docker_utils",
        ":recorder",
        ":tools",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "scripts/src",
    strip_prefix = strip_prefix.from_root("scripts"),
)

pkg_filegroup(
    name = "package_scripts",
    srcs = [":package_scripts_data"],
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        "package_scripts",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

filegroup(
    name = "tools",
    srcs = [
        "record_message.py",
        "record_map_data.py",
        "record_play_pnc.sh",
    ],
)

filegroup(
    name = "bootstrap",
    srcs = [
        "bootstrap.sh",
        "cyber_launch.sh",
        "runtime_env.sh",
    ],
)

filegroup(
    name = "recorder",
    srcs = [
        "recording_launcher.py",
    ],
)

filegroup(
    name = "docker_utils",
    srcs = [
        ":docker_start_user.sh",
    ],
)

filegroup(
    name = "apollo_base",
    srcs = [
        "apollo.bashrc",
        "apollo_base.sh",
        "common.bashrc",
    ],
)
