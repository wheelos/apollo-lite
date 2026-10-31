load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "strategy",
    deps = [
        ":routing_a_star_strategy",
    ],
)

cc_library(
    name = "routing_a_star_strategy",
    srcs = ["a_star_strategy.cc"],
    hdrs = [
        "a_star_strategy.h",
        "strategy.h",
    ],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/routing/graph",
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
