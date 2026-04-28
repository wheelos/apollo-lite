"""Bzlmod extension for local GStreamer installation."""


def _gstreamer_repository_impl(repository_ctx):
    default_sysroot = repository_ctx.attr.sysroot_dir
    sysroot_dir = repository_ctx.os.environ.get("APOLLO_SYSROOT_DIR", default_sysroot)

    include_dir = repository_ctx.path(sysroot_dir + "/include")
    lib_dir = repository_ctx.path(sysroot_dir + "/lib")

    gst_include = include_dir.join("gstreamer-1.0")
    glib_include = include_dir.join("glib-2.0")

    if not gst_include.exists:
        fail("GStreamer include directory not found: {}".format(gst_include))
    if not glib_include.exists:
        fail("GLib include directory not found: {}".format(glib_include))

    # Expose headers under include/ as the BUILD expects
    repository_ctx.symlink(str(gst_include), "include/gstreamer-1.0")
    repository_ctx.symlink(str(glib_include), "include/glib-2.0")

    # Platform-specific glib headers (e.g. /usr/lib/x86_64-linux-gnu/glib-2.0/include)
    arch_x86 = lib_dir.join("x86_64-linux-gnu/glib-2.0/include")
    arch_arm = lib_dir.join("aarch64-linux-gnu/glib-2.0/include")
    if arch_x86.exists:
        repository_ctx.symlink(str(lib_dir.join("x86_64-linux-gnu/glib-2.0/include")), "lib/x86_64-linux-gnu/glib-2.0/include")
    if arch_arm.exists:
        repository_ctx.symlink(str(lib_dir.join("aarch64-linux-gnu/glib-2.0/include")), "lib/aarch64-linux-gnu/glib-2.0/include")

    # Provide a self-contained BUILD.bazel so targets like @gstreamer//:gstreamer exist
    build_bazel = r"""
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "gstreamer",
    includes = [
        "include/gstreamer-1.0",
        "include/glib-2.0",
    ] + select({
        "@platforms//cpu:aarch64": [
            "lib/aarch64-linux-gnu/glib-2.0/include",
        ],
        "@platforms//cpu:x86_64": [
            "lib/x86_64-linux-gnu/glib-2.0/include",
        ],
        "//conditions:default": [],
    }),
    linkopts = [
        "-lgstapp-1.0",
        "-lgstbase-1.0",
        "-lgstreamer-1.0",
        "-lgobject-2.0",
        "-lglib-2.0",
        "-lgio-2.0",
        "-lgmodule-2.0",
    ],
    linkstatic = False,
)
"""
    repository_ctx.file("BUILD.bazel", build_bazel)


gstreamer_repository = repository_rule(
    implementation = _gstreamer_repository_impl,
    attrs = {
        "sysroot_dir": attr.string(default = "/usr"),
    },
    environ = ["APOLLO_SYSROOT_DIR"],
)


def _gstreamer_extension_impl(_ctx):
    gstreamer_repository(name = "gstreamer")


gstreamer = module_extension(
    implementation = _gstreamer_extension_impl,
)
