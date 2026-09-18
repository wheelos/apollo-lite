load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "tf2_headers",
    hdrs = glob([
        "include/geometry_msgs/**",
        "include/tf2_msgs/**",
        "include/tf2/**",
    ]),
    strip_include_prefix = "include",
)

cc_library(
    name = "tf2",
    srcs = [
        "src/buffer_core.cpp",
        "src/cache.cpp",
        "src/static_cache.cpp",
        "src/time.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":tf2_headers",
        "@boost.container",
        "@boost.foreach",
        "@boost.function",
        "@boost.lexical_cast",
        "@boost.signals2",
        "@boost.thread",
        "@boost.tuple",
    ],
)

install(
    name = "install",
    data = [
        ":3rd-tf2.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-tf2",
    library_dest = "3rd-tf2/lib",
    targets = [
        ":libtf2.so",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_tf2_hdrs",
        ":install_tf2_src",
    ],
)

install_src_files(
    name = "install_tf2_src",
    dest = "3rd-tf2/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_tf2_hdrs",
    dest = "3rd-tf2/include",
    filter = "*",
    src_dir = ["include"],
)
