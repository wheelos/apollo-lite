"""Bzlmod extension for local GStreamer installation."""


def _first_existing_path(repository_ctx, candidates, description, required = True):
    checked = []
    for candidate in candidates:
        if not candidate or candidate in checked:
            continue
        checked.append(candidate)
        path = repository_ctx.path(candidate)
        if path.exists:
            return path
    if required:
        fail("{} not found. Checked: {}".format(description, ", ".join(checked)))
    return None


def _render_string_list(items, indent):
    return "".join(["{}\"{}\",\n".format(indent, item) for item in items])


def _gstreamer_repository_impl(repository_ctx):
    default_sysroot = repository_ctx.attr.sysroot_dir
    env_sysroot = repository_ctx.os.environ.get("APOLLO_SYSROOT_DIR", "")
    sysroot_candidates = [env_sysroot, default_sysroot, "/usr"]

    gst_include = _first_existing_path(
        repository_ctx,
        [candidate + "/include/gstreamer-1.0" for candidate in sysroot_candidates],
        "GStreamer include directory",
    )
    glib_include = _first_existing_path(
        repository_ctx,
        [candidate + "/include/glib-2.0" for candidate in sysroot_candidates],
        "GLib include directory",
    )

    # Expose headers under include/ as the BUILD expects
    repository_ctx.symlink(str(gst_include), "include/gstreamer-1.0")
    repository_ctx.symlink(str(glib_include), "include/glib-2.0")

    # Platform-specific glib headers (e.g. /usr/lib/x86_64-linux-gnu/glib-2.0/include)
    arch_x86 = _first_existing_path(
        repository_ctx,
        [candidate + "/lib/x86_64-linux-gnu/glib-2.0/include" for candidate in sysroot_candidates],
        "x86_64 GLib platform include directory",
        required = False,
    )
    arch_arm = _first_existing_path(
        repository_ctx,
        [candidate + "/lib/aarch64-linux-gnu/glib-2.0/include" for candidate in sysroot_candidates],
        "aarch64 GLib platform include directory",
        required = False,
    )
    if arch_x86 != None:
        repository_ctx.symlink(str(arch_x86), "lib/x86_64-linux-gnu/glib-2.0/include")
    if arch_arm != None:
        repository_ctx.symlink(str(arch_arm), "lib/aarch64-linux-gnu/glib-2.0/include")

    include_entries = [
        "include/gstreamer-1.0",
        "include/glib-2.0",
    ]
    hdr_patterns = [
        "include/gstreamer-1.0/**/*.h",
        "include/glib-2.0/**/*.h",
    ]
    if arch_x86 != None:
        include_entries.append("lib/x86_64-linux-gnu/glib-2.0/include")
        hdr_patterns.append("lib/x86_64-linux-gnu/glib-2.0/include/**/*.h")
    if arch_arm != None:
        include_entries.append("lib/aarch64-linux-gnu/glib-2.0/include")
        hdr_patterns.append("lib/aarch64-linux-gnu/glib-2.0/include/**/*.h")

    linkopts = []
    for candidate in sysroot_candidates:
        for suffix in ["/lib", "/lib/x86_64-linux-gnu", "/lib/aarch64-linux-gnu"]:
            link_dir = repository_ctx.path(candidate + suffix)
            link_dir_str = str(link_dir)
            if link_dir.exists and link_dir_str not in linkopts:
                linkopts.append("-L{}".format(link_dir_str))
    linkopts.extend([
        "-lgstapp-1.0",
        "-lgstbase-1.0",
        "-lgstreamer-1.0",
        "-lgobject-2.0",
        "-lglib-2.0",
        "-lgio-2.0",
        "-lgmodule-2.0",
    ])

    # Provide a self-contained BUILD.bazel so targets like @gstreamer//:gstreamer exist
    build_bazel = """
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "gstreamer",
    hdrs = glob([
{hdrs}    ]),
    includes = [
{includes}    ],
    linkopts = [
{linkopts}    ],
    linkstatic = False,
)
""".format(
        hdrs = _render_string_list(hdr_patterns, "        "),
        includes = _render_string_list(include_entries, "        "),
        linkopts = _render_string_list(linkopts, "        "),
    )
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
