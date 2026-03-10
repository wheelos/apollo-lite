"""Bzlmod extension for local Qt5 installation."""

def _get_cpu_arch(repository_ctx):
    """Get CPU architecture using uname -m."""
    result = repository_ctx.execute(["uname", "-m"])
    if result.return_code == 0:
        return result.stdout.strip()
    return "unknown"

def _is_arm64(repository_ctx):
    arch = _get_cpu_arch(repository_ctx)
    return arch in ["aarch64", "arm64"]

def _qt_repository_impl(repository_ctx):
    default_qt_root = repository_ctx.attr.qt_root
    qt_root = repository_ctx.os.environ.get("APOLLO_QT5_ROOT", default_qt_root)

    include_dir = repository_ctx.path(qt_root + "/include")
    lib_dir = repository_ctx.path(qt_root + "/lib")

    if not include_dir.exists:
        fail("Qt include directory not found: {}".format(include_dir))

    # For aarch64, Qt5 libs are in /usr/lib/aarch64-linux-gnu (system path)
    # and /usr/local/qt5/lib may not exist. Skip lib_dir check for aarch64.
    # qt.BUILD handles the correct library path via select().
    if not _is_arm64(repository_ctx):
        if not lib_dir.exists:
            fail("Qt lib directory not found: {}".format(lib_dir))

    repository_ctx.symlink(str(include_dir) + "/QtCore", "QtCore")
    repository_ctx.symlink(str(include_dir) + "/QtGui", "QtGui")
    repository_ctx.symlink(str(include_dir) + "/QtWidgets", "QtWidgets")
    repository_ctx.symlink(str(include_dir) + "/QtOpenGL", "QtOpenGL")
    repository_ctx.symlink(Label("//third_party/qt5:qt.BUILD"), "BUILD.bazel")

qt_repository = repository_rule(
    implementation = _qt_repository_impl,
    attrs = {
        "qt_root": attr.string(default = "/usr/local/qt5"),
    },
    environ = ["APOLLO_QT5_ROOT"],
)

def _qt_extension_impl(_ctx):
    qt_repository(name = "qt")

qt = module_extension(
    implementation = _qt_extension_impl,
)
