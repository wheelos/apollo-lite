load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

install(
    name = "install",
    data = [
        "cyberfile.xml",
    ],
    data_dest = "3rd-localization-msf",
)

# The bundled MSF library was removed to reduce repository size. Re-enable
# source installation after the MSF refactor provides a replacement artifact.
