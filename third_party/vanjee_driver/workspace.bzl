"""load vanjee driver"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "vanjee_driver",
        sha256 = "c4a9f6a5311600044e5d7211bfcec3b06d86acda9456b060d0c0e054b9f833bb",
        build_file = clean_dep("//third_party/vanjee_driver:vanjee.BUILD"),
        strip_prefix = "vanjee_driver-2.2.9",
        urls = [
            "https://github.com/wheelos/vanjee_driver_sdk/archive/refs/tags/v2.2.9.tar.gz",
        ],
    )
