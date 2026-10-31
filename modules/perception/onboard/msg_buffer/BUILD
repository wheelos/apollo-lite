load("@rules_cc//cc:defs.bzl", "cc_library")

# DO NOT MODIFY
# THIS FILE IS GENERATED AUTOMATICALLY TO HELP WRITE BUILD FILE
# DELETE THIS FILE WHEN YOU HAVE CREATED THE COMPLETE BUILD FILE

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msg_buffer",
    srcs = ["msg_buffer.cc"],
    hdrs = ["msg_buffer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@boost.circular_buffer",
        "@core//cyber",
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
