"""Bzlmod extension for local Qt5 installation."""

def _get_cpu_arch(repository_ctx):
    return repository_ctx.os.arch

def _is_arm64(repository_ctx):
    arch = _get_cpu_arch(repository_ctx)
    return arch in ["aarch64", "arm64"]

def _qt_repository_impl(repository_ctx):
    default_qt_root = repository_ctx.attr.qt_root
    qt_root = repository_ctx.getenv("APOLLO_QT5_ROOT", default_qt_root)

    include_dir = repository_ctx.path(qt_root + "/include")
    lib_dir = repository_ctx.path(qt_root + "/lib")

    if not include_dir.exists:
        fail("Qt include directory not found: {}".format(include_dir))

    bin_dir = repository_ctx.path(qt_root + "/bin")

    # The Orin installer supplies system Qt with only an include-prefix symlink.
    if _is_arm64(repository_ctx) and not lib_dir.exists:
        lib_dir = repository_ctx.path("/usr/lib/aarch64-linux-gnu")
    if _is_arm64(repository_ctx) and not bin_dir.exists:
        bin_dir = repository_ctx.path("/usr/lib/qt5/bin")

    for component in ["QtCore", "QtGui", "QtWidgets", "QtOpenGL"]:
        headers = repository_ctx.path(str(include_dir) + "/" + component)
        library = repository_ctx.path(str(lib_dir) + "/lib" + component.replace("Qt", "Qt5", 1) + ".so")
        if not headers.exists or not library.exists:
            fail("Qt SDK component missing: {} or {}".format(headers, library))
        repository_ctx.symlink(headers, component)
        repository_ctx.symlink(library, "lib/" + library.basename)
    for tool in ["moc", "uic", "rcc"]:
        executable = repository_ctx.path(str(bin_dir) + "/" + tool)
        if not executable.exists:
            fail("Qt SDK tool not found: {}".format(executable))
        repository_ctx.symlink(executable, "bin/" + tool)
    repository_ctx.symlink(Label("//third_party/qt5:qt.BUILD"), "BUILD.bazel")

qt_repository = repository_rule(
    implementation = _qt_repository_impl,
    attrs = {
        "qt_root": attr.string(default = "/usr/local/qt5"),
    },
    environ = ["APOLLO_QT5_ROOT"],
    local = True,
    configure = True,
)

def _qt_extension_impl(_ctx):
    qt_repository(name = "qt")

qt = module_extension(
    implementation = _qt_extension_impl,
)
