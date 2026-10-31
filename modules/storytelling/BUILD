load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_manager",
    srcs = ["frame_manager.cc"],
    hdrs = ["frame_manager.h"],
    deps = [
        "@wheelos_common//modules/common/monitor_log",
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

pkg_files(
    name = "package_files_data",
    srcs = [
        ":runtime_data",
        ":storytelling.BUILD",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "storytelling",
    strip_prefix = strip_prefix.from_root("modules/storytelling"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libstorytelling_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "storytelling/lib",
    strip_prefix = strip_prefix.from_root("modules/storytelling"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        ":pb_hdrs",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_hdrs_data",
    srcs = ["//modules/storytelling/proto:storytelling_config_cc_proto"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "storytelling/include",
    strip_prefix = strip_prefix.from_root("modules/storytelling"),
)

pkg_filegroup(
    name = "pb_hdrs",
    srcs = [":pb_hdrs_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_storytelling_hdrs",
        ":package_storytelling_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_storytelling_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/storytelling/common:exported_source_files",
        "//modules/storytelling/proto:exported_source_files",
        "//modules/storytelling/story_tellers:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "storytelling/src",
    strip_prefix = strip_prefix.from_root("modules/storytelling"),
)

pkg_filegroup(
    name = "package_storytelling_src",
    srcs = [":package_storytelling_src_data"],
)

pkg_files(
    name = "package_storytelling_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/storytelling/common:exported_source_headers",
        "//modules/storytelling/story_tellers:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "storytelling/include",
    strip_prefix = strip_prefix.from_root("modules/storytelling"),
)

pkg_filegroup(
    name = "package_storytelling_hdrs",
    srcs = [":package_storytelling_hdrs_data"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)
