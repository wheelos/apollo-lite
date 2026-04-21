load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_ransac",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = ["i_ransac.h"],
)

cpplint()
