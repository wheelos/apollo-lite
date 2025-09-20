load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rs_driver",
    hdrs = glob(["src/rs_driver/**/*.hpp"]),
    includes = [
        "src",
    ],
    linkopts = [
        "-lpcap",
    ],
)
