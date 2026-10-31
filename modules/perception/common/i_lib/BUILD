load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_lib",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/common/i_lib/algorithm:i_sort",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/da:i_ransac",
        "//modules/perception/common/i_lib/geometry:i_plane",
        "//modules/perception/common/i_lib/pc:i_ground",
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
