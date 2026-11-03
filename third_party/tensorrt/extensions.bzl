"""Compatibility exports; Lane owns the shared TensorRT SDK integration."""

load(
    "@wheelos_common//third_party/tensorrt:extensions.bzl",
    _tensorrt_configure = "tensorrt_configure",
    _tensorrt_repository = "tensorrt_repository",
)

tensorrt_configure = _tensorrt_configure
tensorrt_repository = _tensorrt_repository
