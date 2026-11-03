load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [":sim_control"],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "dreamview/bin",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend/sim_control_manager"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "sim_control_manager",
    srcs = ["sim_control_manager.cc"],
    hdrs = [
        "sim_control_manager.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//modules/dreamview/backend/sim_control_manager/core:dynamic_model_factory",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "@com_github_nlohmann_json//:json",
        "@core//cyber",
    ],
)

cc_binary(
    name = "sim_control",
    srcs = ["main.cc"],
    copts = ['-DMODULE_NAME=\\"sim_control\\"'],
    deps = [
        ":sim_control_manager",
        "@core//cyber",
    ],
)

# todo: add test file
# cc_test(
#     name = "sim_control_test",
#     size = "small",
#     srcs = ["sim_control_test.cc"],
#     data = [
#         "//modules/dreamview/backend/testdata",
#     ],
#     deps = [
#         ":sim_control",
#         "@core//cyber",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

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
