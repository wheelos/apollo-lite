load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obj_mapper",
    srcs = ["obj_mapper.cc"],
    hdrs = ["obj_mapper.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/common/proto:object_template_meta_schema_cc_proto",
        "@core//cyber",
    ],
)

cc_library(
    name = "multicue_obstacle_transformer",
    srcs = ["multicue_obstacle_transformer.cc"],
    hdrs = ["multicue_obstacle_transformer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":obj_mapper",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:multicue_cc_proto",
        "@core//cyber",
    ],
    alwayslink = True,
)

cpplint()
