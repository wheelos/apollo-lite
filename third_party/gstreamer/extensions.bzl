"""Bzlmod extension for the local GStreamer development packages."""

_PKG_CONFIG_ENV = [
    "PATH",
    "PKG_CONFIG_PATH",
    "PKG_CONFIG_LIBDIR",
    "PKG_CONFIG_SYSROOT_DIR",
]

def _pkg_config(repository_ctx, option, environment):
    executable = repository_ctx.which("pkg-config")
    if executable == None:
        fail("pkg-config not found on PATH. Install pkg-config and the GStreamer development packages.")
    result = repository_ctx.execute(
        ["/usr/bin/env", "-i"] +
        ["{}={}".format(name, environment[name]) for name in sorted(environment)] +
        [str(executable), option, "gstreamer-app-1.0", "gstreamer-video-1.0"],
        quiet = True,
    )
    if result.return_code != 0:
        fail("GStreamer pkg-config {} failed:\n{}".format(option, result.stderr))
    return [flag for flag in result.stdout.replace("\n", " ").replace("\t", " ").split(" ") if flag]

def _gstreamer_repository_impl(repository_ctx):
    # APOLLO_SYSROOT_DIR is an installation prefix, not a cross-compilation sysroot.
    prefix = repository_ctx.getenv("APOLLO_SYSROOT_DIR", repository_ctx.attr.sysroot_dir)
    environment = {name: repository_ctx.getenv(name, "") for name in _PKG_CONFIG_ENV}
    environment["LC_ALL"] = "C"
    environment["PKG_CONFIG_DISABLE_UNINSTALLED"] = "1"
    prefix_paths = [
        prefix + "/lib/x86_64-linux-gnu/pkgconfig",
        prefix + "/lib/aarch64-linux-gnu/pkgconfig",
        prefix + "/lib/pkgconfig",
        prefix + "/share/pkgconfig",
    ]
    environment["PKG_CONFIG_PATH"] = ":".join(
        [path for path in [environment["PKG_CONFIG_PATH"]] + prefix_paths if path],
    )

    # An unset PKG_CONFIG_LIBDIR must retain pkg-config's compiled-in system paths.
    if not environment["PKG_CONFIG_LIBDIR"]:
        environment.pop("PKG_CONFIG_LIBDIR")

    includes = []
    headers = []
    copts = []
    for flag in _pkg_config(repository_ctx, "--cflags", environment):
        if flag.startswith("-I"):
            include_dir = flag[2:]
            if not include_dir.startswith("/") or not repository_ctx.path(include_dir).exists:
                fail("Invalid GStreamer pkg-config include directory: {}".format(include_dir))
            if include_dir in includes:
                continue

            # Use flat symlinks: multiarch include paths can be nested under /usr/include.
            repo_path = "include_{}".format(len(includes))
            repository_ctx.symlink(include_dir, repo_path)
            includes.append(include_dir)
            headers.append(repo_path + "/**/*.h")
        else:
            copts.append(flag)
    if not includes:
        fail("GStreamer pkg-config returned no include directories.")

    repository_ctx.file(
        "BUILD.bazel",
        """load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "gstreamer",
    hdrs = glob({headers}),
    copts = {copts},
    includes = {includes},
    linkopts = {linkopts},
    linkstatic = False,
)
""".format(
            headers = repr(headers),
            copts = repr(copts),
            includes = repr(["include_{}".format(i) for i in range(len(includes))]),
            linkopts = repr(_pkg_config(repository_ctx, "--libs", environment)),
        ),
    )

gstreamer_repository = repository_rule(
    implementation = _gstreamer_repository_impl,
    attrs = {
        "sysroot_dir": attr.string(default = "/usr"),
    },
    environ = ["APOLLO_SYSROOT_DIR"] + _PKG_CONFIG_ENV,
    local = True,
    configure = True,
)

def _gstreamer_extension_impl(_ctx):
    gstreamer_repository(name = "gstreamer")

gstreamer = module_extension(
    implementation = _gstreamer_extension_impl,
)
