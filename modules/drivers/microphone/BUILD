load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

MICROPHONE_COPTS = ['-DMODULE_NAME=\\"microphone\\"']

cc_binary(
    name = "libmicrophone_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":microphone_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/microphone",
    library_dest = "drivers/lib/microphone",
    targets = [
        ":libmicrophone_component.so",
    ],
    deps = [":install_lib_top_level"],
)

install(
    name = "install_lib_top_level",
    library_dest = "drivers/lib",
    targets = [
        ":libmicrophone_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "microphone_component_lib",
    srcs = ["microphone_component.cc"],
    hdrs = ["microphone_component.h"],
    copts = MICROPHONE_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/drivers/microphone:respeaker",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_library(
    name = "respeaker",
    srcs = ["respeaker.cc"],
    hdrs = ["respeaker.h"],
    copts = MICROPHONE_COPTS,
    deps = [
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@core//cyber",
        "@portaudio",
    ],
)

cpplint()
