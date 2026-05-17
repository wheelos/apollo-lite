load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_plane",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = [
        "i_line.h",
        "i_plane.h",
        "i_util.h",
    ],
    deps = [
        "@eigen",
    ],
)

cpplint()
