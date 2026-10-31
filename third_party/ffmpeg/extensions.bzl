"""Bzlmod extension for local FFmpeg installation."""

def _ffmpeg_repository_impl(repository_ctx):
    default_sysroot = repository_ctx.attr.sysroot_dir
    sysroot_dir = repository_ctx.getenv("APOLLO_SYSROOT_DIR", default_sysroot)

    include_dir = repository_ctx.path(sysroot_dir + "/include")
    lib_dir = repository_ctx.path(sysroot_dir + "/lib")

    if not include_dir.exists:
        fail("FFmpeg include directory not found: {}".format(include_dir))
    if not lib_dir.exists:
        fail("FFmpeg lib directory not found: {}".format(lib_dir))
    for component in ["libavcodec", "libavformat", "libavutil", "libswscale"]:
        if not repository_ctx.path(str(include_dir) + "/" + component).exists:
            fail("FFmpeg headers not found: {}/{}".format(include_dir, component))
        if not repository_ctx.path(str(lib_dir) + "/" + component + ".so").exists:
            fail("FFmpeg library not found: {}/{}.so".format(lib_dir, component))

    repository_ctx.symlink(lib_dir, "lib")
    repository_ctx.symlink(str(include_dir) + "/libavcodec", "libavcodec")
    repository_ctx.symlink(str(include_dir) + "/libavformat", "libavformat")
    repository_ctx.symlink(str(include_dir) + "/libavutil", "libavutil")
    repository_ctx.symlink(str(include_dir) + "/libswscale", "libswscale")
    repository_ctx.symlink(Label("//third_party/ffmpeg:ffmpeg.BUILD"), "BUILD.bazel")

ffmpeg_repository = repository_rule(
    implementation = _ffmpeg_repository_impl,
    attrs = {
        "sysroot_dir": attr.string(default = "/opt/apollo/sysroot"),
    },
    environ = ["APOLLO_SYSROOT_DIR"],
    local = True,
    configure = True,
)

def _ffmpeg_extension_impl(_ctx):
    ffmpeg_repository(name = "ffmpeg")

ffmpeg = module_extension(
    implementation = _ffmpeg_extension_impl,
)
