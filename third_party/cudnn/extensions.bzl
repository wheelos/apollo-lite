"""Dependencies for cudnn."""

def _is_linux(repository_ctx):
    return repository_ctx.os.name.startswith("linux")

def _get_cpu_arch(repository_ctx):
    """Get CPU architecture using uname -m."""
    result = repository_ctx.execute(["uname", "-m"])
    if result.return_code == 0:
        return result.stdout.strip()
    return "unknown"

def _is_x86_64(repository_ctx):
    arch = _get_cpu_arch(repository_ctx)
    return arch in ["amd64", "x86_64"]

def _is_arm64(repository_ctx):
    arch = _get_cpu_arch(repository_ctx)
    return arch in ["aarch64", "arm64"]

def _symlink_file_if_exists(repository_ctx, src_dir, file, dst_dir):
    src_path = repository_ctx.path(src_dir + "/" + file)
    if src_path.exists:
        repository_ctx.symlink(src_dir + "/" + file, dst_dir + "/" + file)

def _get_cudnn_version(repository_ctx, include_path):
    """Detect cuDNN version by checking cudnn_version.h."""
    version_file = repository_ctx.path(include_path + "/cudnn_version.h")
    if not version_file.exists:
        version_file = repository_ctx.path(include_path + "/cudnn.h")
    if version_file.exists:
        # Try to read version from header
        result = repository_ctx.execute(["cat", str(version_file)])
        if result.return_code == 0:
            content = result.stdout
            # Look for CUDNN_MAJOR version
            if "CUDNN_MAJOR 9" in content or "CUDNN_MAJOR=9" in content:
                return 9
            elif "CUDNN_MAJOR 8" in content or "CUDNN_MAJOR=8" in content:
                return 8
    # Default to version 9 if cannot determine
    return 9

def _symlink_cudnn8_headers(repository_ctx, include_path, dst_dir):
    """Symlink cuDNN 8 header files."""
    for name in [
        "cudnn.h",
        "cudnn_backend.h",
        "cudnn_ops_infer.h",
        "cudnn_ops_train.h",
        "cudnn_cnn_infer.h",
        "cudnn_cnn_train.h",
        "cudnn_adv_infer.h",
        "cudnn_adv_train.h",
        "cudnn_version.h",
    ]:
        _symlink_file_if_exists(repository_ctx, include_path, name, dst_dir)

def _symlink_cudnn8_libraries(repository_ctx, library_path, dst_dir):
    """Symlink cuDNN 8 library files."""
    for name in [
        "libcudnn.so",
        "libcudnn_ops_infer.so",
        "libcudnn_ops_train.so",
        "libcudnn_cnn_infer.so",
        "libcudnn_cnn_train.so",
        "libcudnn_adv_infer.so",
        "libcudnn_adv_train.so",
    ]:
        _symlink_file_if_exists(repository_ctx, library_path, name, dst_dir)

def _symlink_cudnn9_headers(repository_ctx, include_path, dst_dir):
    """Symlink cuDNN 9 header files."""
    for name in [
        "cudnn.h",
        "cudnn_adv.h",
        "cudnn_backend.h",
        "cudnn_cnn.h",
        "cudnn_graph.h",
        "cudnn_ops.h",
        "cudnn_version.h",
    ]:
        _symlink_file_if_exists(repository_ctx, include_path, name, dst_dir)

def _symlink_cudnn9_libraries(repository_ctx, library_path, dst_dir):
    """Symlink cuDNN 9 library files."""
    for name in [
        "libcudnn.so",
        "libcudnn_adv.so",
        "libcudnn_cnn.so",
        "libcudnn_graph.so",
        "libcudnn_ops.so",
        "libheuristic.so",
        "libcudnn_engines_precompiled.so",
        "libcudnn_engines_runtime_compiled.so",
    ]:
        _symlink_file_if_exists(repository_ctx, library_path, name, dst_dir)

def _cudnn_impl(repository_ctx):
    # Path to cudnn is
    # - taken from CUDNN_INSTALL_PATH environment variable or
    # - defaults to '/usr'
    cudnn_include_path = None
    cudnn_library_path = None

    if _is_linux(repository_ctx):
        cudnn_install_path = repository_ctx.os.environ.get("CUDNN_INSTALL_PATH", "/usr")
        guess_include_paths = [
            cudnn_install_path + "/include",
            "/usr/include",
        ]
        for include_path in guess_include_paths:
            if repository_ctx.path(include_path + "/cudnn.h").exists:
                cudnn_include_path = include_path
                break
        guess_library_paths = [
            cudnn_install_path + "/lib64",
            cudnn_install_path + "/lib",
            "/usr/lib/x86_64-linux-gnu",
            "/lib/aarch64-linux-gnu",
        ]
        for library_path in guess_library_paths:
            if repository_ctx.path(library_path + "/libcudnn.so").exists:
                cudnn_library_path = library_path
                break
    else:
        fail("Unsupported OS")

    if (
        cudnn_include_path and repository_ctx.path(cudnn_include_path).exists and
        cudnn_library_path and repository_ctx.path(cudnn_library_path).exists
    ):
        # Detect cuDNN version
        cudnn_version = _get_cudnn_version(repository_ctx, cudnn_include_path)

        if cudnn_version == 8:
            repository_ctx.symlink(Label("//third_party/cudnn:BUILD.cudnn8"), "BUILD")
            _symlink_cudnn8_headers(repository_ctx, cudnn_include_path, "cudnn/include")
            _symlink_cudnn8_libraries(repository_ctx, cudnn_library_path, "cudnn/lib")
        else:
            # Default to cuDNN 9
            repository_ctx.symlink(Label("//third_party/cudnn:BUILD.cudnn9"), "BUILD")
            _symlink_cudnn9_headers(repository_ctx, cudnn_include_path, "cudnn/include")
            _symlink_cudnn9_libraries(repository_ctx, cudnn_library_path, "cudnn/lib")

    else:
        # dummy
        repository_ctx.file("BUILD", content = "")

_cudnn = repository_rule(
    implementation = _cudnn_impl,
    environ = ["CUDNN_INSTALL_PATH", "CUDA_PATH"],
)

def _cudnn_extension_impl(_ctx):
    _cudnn(name = "local_config_cudnn")

cudnn = module_extension(
    implementation = _cudnn_extension_impl,
)
