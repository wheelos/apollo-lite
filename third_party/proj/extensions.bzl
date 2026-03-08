"""Bzlmod extension for local PROJ installation."""

def _proj_repository_impl(repository_ctx):
    default_sysroot = repository_ctx.attr.sysroot_dir
    sysroot_dir = repository_ctx.os.environ.get("APOLLO_SYSROOT_DIR", default_sysroot)

    include_dir = repository_ctx.path(sysroot_dir + "/include")
    lib_dir = repository_ctx.path(sysroot_dir + "/lib")

    if not include_dir.exists:
        fail("PROJ include directory not found: {}".format(include_dir))
    if not lib_dir.exists:
        fail("PROJ lib directory not found: {}".format(lib_dir))

    repository_ctx.symlink(str(include_dir), "include")
    repository_ctx.symlink(str(lib_dir), "lib")
    repository_ctx.symlink(Label("//third_party/proj:proj.BUILD"), "BUILD.bazel")

proj_repository = repository_rule(
    implementation = _proj_repository_impl,
    attrs = {
        "sysroot_dir": attr.string(default = "/opt/apollo/sysroot"),
    },
    environ = ["APOLLO_SYSROOT_DIR"],
)

def _proj_extension_impl(_ctx):
    proj_repository(name = "proj")

proj = module_extension(
    implementation = _proj_extension_impl,
)
