load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vanjee_driver",
    hdrs = glob(["src/vanjee_driver/**/*.hpp"]),
    includes = [
        "src",
    ],
    linkopts = [
        "-lpcap",
    ],
)
