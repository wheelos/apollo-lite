load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    hdrs = [
        "i_alloc.h",
        "i_basic.h",
        "i_blas.h",
        "i_constant.h",
        "i_rand.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
