load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":container_scripts",
    ],
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
