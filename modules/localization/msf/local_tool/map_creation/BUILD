load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "localization/bin",
    targets = [
        ":lossless_map_creator",
        ":lossless_map_to_lossy_map",
        ":poses_interpolator",
    ],
)

cc_binary(
    name = "lossless_map_creator",
    srcs = ["lossless_map_creator.cc"],
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "@boost.filesystem",
        "@boost.program_options",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = ["lossless_map_to_lossy_map.cc"],
    linkstatic = True,
    deps = [
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "@boost.filesystem",
        "@boost.program_options",
    ],
)

cc_binary(
    name = "poses_interpolator",
    srcs = ["poses_interpolator.cc"],
    linkstatic = True,
    deps = [
        "//modules/localization/msf/local_tool/map_creation/poses_interpolation:poses_interpolation_lib",
        "@boost.program_options",
    ],
)

cpplint()
