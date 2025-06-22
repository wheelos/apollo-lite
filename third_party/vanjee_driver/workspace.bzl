"""load vanjee driver"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "vanjee_driver",
        # sha256 = "3d7ccacf0fe42d068f2c7a58bbf08b4e947c65b9651b60c268ba91cf8c42a4e5",
        # build_file = clean_dep("//third_party/vanjee_driver:vanjee.BUILD"),
        # strip_prefix = "vanjee_driver-1.10.1",
        # urls = [
        #     "https://github.com/wheelos/vanjee_driver_sdk/archive/refs/tags/v1.10.1.tar.gz",
        # ],
        sha256 = "69711732b3c84b3a19c2c710deeb90d2f339c73cf4c7657810d9b462b3a0d15e",
        build_file = clean_dep("//third_party/vanjee_driver:vanjee.BUILD"),
        strip_prefix = "vanjee_driver-2.0.15",
        urls = [
            "https://github.com/wheelos/vanjee_driver_sdk/archive/refs/tags/v2.0.15.tar.gz",
        ],
    )
