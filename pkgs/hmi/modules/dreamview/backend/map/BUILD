load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_archives",
    srcs = [":map_service"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend/map"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "map_service",
    srcs = [
        "map_service.cc",
    ],
    hdrs = ["map_service.h"],
    deps = [
        "@wheelos_common//modules/common/map:map_selection",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@boost.thread",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_test(
    name = "map_service_test",
    size = "small",
    srcs = ["map_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":map_service",
        "@wheelos_common//modules/common/map:map_selection",
        "@com_google_googletest//:gtest_main",
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
