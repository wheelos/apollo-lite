load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ipopt",
    hdrs = glob(["include/coin/**/*.hpp", "include/coin/**/*.h"]),
    includes = ["include"],
    linkopts = [
        "-lipopt",
    ],
)
