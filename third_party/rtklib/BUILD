load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtklib",
    srcs = [
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    hdrs = ["rtklib.h"],
    linkstatic = True,
    alwayslink = True,
)

cc_binary(
    name = "librtklib.so",
    linkshared = True,
    linkstatic = False,
    deps = [":rtklib"],
)

install(
    name = "install",
    data = [
        ":3rd-rtklib.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-rtklib",
    library_dest = "3rd-rtklib/lib",
    targets = ["librtklib.so"],
)

install_src_files(
    name = "headers",
    dest = "3rd-rtklib/include",
    filter = "*.h",
    src_dir = ["."],
)

install_src_files(
    name = "install_src",
    dest = "3rd-rtklib/src",
    filter = "*",
    src_dir = ["."],
    deps = [
        ":headers",
    ],
)
