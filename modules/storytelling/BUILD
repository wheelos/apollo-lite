load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_manager",
    srcs = ["frame_manager.cc"],
    hdrs = ["frame_manager.h"],
    deps = [
        "//modules/common/monitor_log",
        "@core//cyber",
    ],
)

cc_library(
    name = "storytelling_lib",
    srcs = ["storytelling.cc"],
    hdrs = ["storytelling.h"],
    copts = ['-DMODULE_NAME=\\"storytelling\\"'],
    deps = [
        "//modules/storytelling/story_tellers:close_to_junction_teller",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libstorytelling_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":storytelling_lib"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
        ":storytelling.BUILD",
    ],
    data_dest = "storytelling",
    library_dest = "storytelling/lib",
    targets = [
        ":libstorytelling_component.so",
    ],
    deps = [
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data = [
        "//modules/storytelling/proto:storytelling_config_cc_proto",
    ],
    data_dest = "storytelling/include",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_storytelling_hdrs",
        ":install_storytelling_src",
    ],
)

install_src_files(
    name = "install_storytelling_src",
    dest = "storytelling/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_storytelling_hdrs",
    dest = "storytelling/include",
    filter = "*.h",
    src_dir = ["."],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
