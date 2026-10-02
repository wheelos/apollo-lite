load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ndt_map_creator",
    srcs = ["ndt_map_creator.cc"],
    deps = [
        "//modules/localization/common/pointcloud_io:common_io",
        "//modules/ndt_localization/map_support/base_map",
        "//modules/ndt_localization/map_support/ndt_map",
        "//modules/ndt_localization/map_support/ndt_map:ndt_map_pool",
        "//modules/ndt_localization/map_support/util",
        "//modules/ndt_localization/map_support/util:system_utility",
        "@boost.filesystem",
        "@boost.program_options",
        "@boost.random",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@eigen",
    ],
)

install(
    name = "install",
    runtime_dest = "ndt_localization/bin",
    targets = [
        ":ndt_map_creator",
    ],
)

cpplint()
