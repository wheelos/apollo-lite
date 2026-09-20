load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "map/lib",
    targets = [":opendrive_adapter"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "opendrive_adapter",
    srcs = [
        "opendrive_adapter.cc",
    ],
    hdrs = [
        "opendrive_adapter.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_organizer",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/hdmap/adapter/xml_parser",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@proj",
        "@tinyxml2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_cc_proto",
    ],
)

cc_library(
    name = "proto_organizer",
    srcs = ["proto_organizer.cc"],
    hdrs = ["proto_organizer.h"],
    linkstatic = True,
    visibility = ["//visibility:private"],
    deps = [
        "//modules/common/math",
        "//modules/common/util",
        "//modules/map/hdmap/adapter/xml_parser",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
