load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avcodec",
    hdrs = glob(["libavcodec/*.h"]),
    includes = ["."],
    deps = [
        ":avcodec_shared",
        ":avutil",
    ],
)

cc_library(
    name = "avformat",
    hdrs = glob(["libavformat/*.h"]),
    includes = ["."],
    deps = [
        ":avcodec",
        ":avformat_shared",
        ":avutil",
    ],
)

cc_library(
    name = "swscale",
    hdrs = glob(["libswscale/*.h"]),
    includes = ["."],
    deps = [
        ":avutil",
        ":swscale_shared",
    ],
)

cc_library(
    name = "avutil",
    hdrs = glob(["libavutil/*.h"]),
    includes = ["."],
    deps = [":avutil_shared"],
)

cc_import(
    name = "avcodec_shared",
    shared_library = "lib/libavcodec.so",
)

cc_import(
    name = "avformat_shared",
    shared_library = "lib/libavformat.so",
)

cc_import(
    name = "avutil_shared",
    shared_library = "lib/libavutil.so",
)

cc_import(
    name = "swscale_shared",
    shared_library = "lib/libswscale.so",
)
