"""load vanjee driver"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "rs_driver",
        # sha256 = "1b01c218abbf239a7d64101be9661a1ea191542f064fa538de5f069a60711740",
        # build_file = clean_dep("//third_party/rs_driver:rs_driver.BUILD"),
        # strip_prefix = "rs_driver-master",
        # urls = [
        #     "file:///tmp/rs_driver-master.tar.gz",
        # ],
        sha256 = "3e20c89734f6672f1c8d9ee783f29cc28b4661b04d16a9ae50dc48610564bf7c",
        build_file = clean_dep("//third_party/rs_driver:rs_driver.BUILD"),
        strip_prefix = "rs_driver-1.5.18",
        urls = [
            "https://github.com/RoboSense-LiDAR/rs_driver/archive/refs/tags/v1.5.18.tar.gz",
        ],
    )
