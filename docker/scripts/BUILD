load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_data",
    srcs = [":container_scripts"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "docker-scripts",
    strip_prefix = strip_prefix.from_root("docker/scripts"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_data"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

filegroup(
    name = "container_scripts",
    srcs = [
        "container_selection.sh",
        "entrypoint.sh",
        "env_setup.sh",
        "welcome.sh",
        "whl.sh",
    ],
)
