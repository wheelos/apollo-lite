load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtklib",
    srcs = [
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    hdrs = ["rtklib.h"],
    linkstatic = True,
    alwayslink = True,
)

cc_binary(
    name = "librtklib.so",
    linkshared = True,
    linkstatic = False,
    deps = [":rtklib"],
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":BUILD",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "3rd-rtklib",
    strip_prefix = strip_prefix.from_root("third_party/rtklib"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = ["librtklib.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "3rd-rtklib/lib",
    strip_prefix = strip_prefix.from_root("third_party/rtklib"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "headers_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    attributes = pkg_attributes(mode = "0644"),
    prefix = "3rd-rtklib/include",
    strip_prefix = strip_prefix.from_root("third_party/rtklib"),
)

pkg_filegroup(
    name = "headers",
    srcs = [":headers_data"],
)

pkg_files(
    name = "source_files_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    attributes = pkg_attributes(mode = "0644"),
    prefix = "3rd-rtklib/src",
    strip_prefix = strip_prefix.from_root("third_party/rtklib"),
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":headers",
        ":source_files_data",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)
