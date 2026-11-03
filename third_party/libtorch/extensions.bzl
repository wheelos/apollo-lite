"""Compatibility exports for the shared LibTorch repository integration."""

load(
    "@wheelos_common//third_party/libtorch:extensions.bzl",
    _libtorch_cpu_repository = "libtorch_cpu_repository",
    _libtorch_ext = "libtorch_ext",
    _libtorch_gpu_repository = "libtorch_gpu_repository",
)

libtorch_cpu_repository = _libtorch_cpu_repository
libtorch_gpu_repository = _libtorch_gpu_repository
libtorch_ext = _libtorch_ext
