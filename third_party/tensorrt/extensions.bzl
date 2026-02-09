def _tensorrt_repository_impl(repository_ctx):
    # 1. 获取用户指定的路径，或者默认为 Jetson 的系统路径
    trt_path = repository_ctx.attr.path

    # 2. 检查路径是否存在
    if not repository_ctx.path(trt_path).exists:
         # 如果默认路径不存在，尝试探测 /usr
         if trt_path == "/usr/local/tensorrt":
             trt_path = "/usr"
         else:
             fail("TensorRT path not found at: %s" % trt_path)

    # 3. 将宿主机的 TensorRT 目录映射到 Bazel 仓库内部

    # 注意：在 Jetson 上，头文件可能直接混在 /usr/include 中，也可能在 /usr/include/aarch64-linux-gnu
    # 为了简化，我们直接链接这个根路径，并在 BUILD 文件中通过 include_prefix 调整
    repository_ctx.symlink(trt_path, "tensorrt_root")

    build_content = """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt",
    hdrs = glob([
        "tensorrt_root/include/aarch64-linux-gnu/NvInfer*.h",
        "tensorrt_root/include/aarch64-linux-gnu/NvUtils.h",
        "tensorrt_root/include/NvInfer*.h",
        "tensorrt_root/include/NvUtils.h",
    ]),
    srcs = glob([
        "tensorrt_root/lib/aarch64-linux-gnu/libnvinfer.so*",
        "tensorrt_root/lib/aarch64-linux-gnu/libnvinfer_plugin.so*",
        "tensorrt_root/lib/libnvinfer.so*",
        "tensorrt_root/lib/libnvinfer_plugin.so*",
    ]),
    # 这里非常重要：告诉编译器去哪里找头文件
    # strip_include_prefix 去掉前面的 "tensorrt_root"
    includes = [
        "tensorrt_root/include",
        "tensorrt_root/include/aarch64-linux-gnu"
    ],
    strip_include_prefix = "tensorrt_root",
    linkstatic = 0,
)
"""
    repository_ctx.file("BUILD", build_content)

tensorrt_repository = repository_rule(
    implementation = _tensorrt_repository_impl,
    attrs = {
        "path": attr.string(default = "/usr"), # Jetson 默认是系统路径
    },
)

def _tensorrt_configure_impl(ctx):
    # 这里可以添加逻辑从环境变量读取路径，比如 ENV["TENSORRT_PATH"]
    tensorrt_repository(
        name = "local_config_tensorrt",
        path = "/usr", # 针对 Jetson/Ubuntu 系统安装
        # 如果是 x86 独立安装包，可能是 "/usr/local/tensorrt"
    )

tensorrt_configure = module_extension(implementation = _tensorrt_configure_impl)
