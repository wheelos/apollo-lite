def _libtorch_repo_impl(rctx, is_gpu):
    build_file = rctx.path(Label("//third_party/libtorch:libtorch_gpu.BUILD")) if is_gpu else rctx.path(Label("//third_party/libtorch:libtorch_cpu.BUILD"))
    
    rctx.symlink("/usr/local/libtorch/include", "include")
    rctx.symlink("/usr/local/libtorch/lib", "lib")
    rctx.symlink(build_file, "BUILD.bazel")

def _libtorch_cpu_repo_impl(rctx):
    _libtorch_repo_impl(rctx, False)

def _libtorch_gpu_repo_impl(rctx):
    _libtorch_repo_impl(rctx, True)

libtorch_cpu_repository = repository_rule(
    implementation = _libtorch_cpu_repo_impl,
    local = True,
)

libtorch_gpu_repository = repository_rule(
    implementation = _libtorch_gpu_repo_impl,
    local = True,
)

def _libtorch_extension_impl(mctx):
    libtorch_cpu_repository(name = "libtorch_cpu")
    libtorch_gpu_repository(name = "libtorch_gpu")

libtorch_ext = module_extension(
    implementation = _libtorch_extension_impl,
)
