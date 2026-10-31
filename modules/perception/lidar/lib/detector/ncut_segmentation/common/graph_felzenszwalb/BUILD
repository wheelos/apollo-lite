load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph_felzenszwalb",
    hdrs = [
        "convolve.h",
        "disjoint_set.h",
        "filter.h",
        "image.h",
        "imconv.h",
        "imutil.h",
        "misc.h",
        "segment_graph.h",
        "segment_image.h",
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
