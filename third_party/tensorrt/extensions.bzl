def _tensorrt_repository_impl(repository_ctx):
    # 针对 Jetpack 6 / Ubuntu 标准布局
    # 我们分别链接 include 和 lib，而不是整个 /usr

    # 1. 映射头文件
    repository_ctx.symlink("/usr/include", "trt_include")

    # 2. 映射库文件 (针对 aarch64)
    repository_ctx.symlink("/usr/lib/aarch64-linux-gnu", "trt_lib")

    build_content = """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt",
    hdrs = glob([
        "trt_include/NvInfer*.h",
        "trt_include/NvUtils.h",
        "trt_include/aarch64-linux-gnu/NvInfer*.h",
    ]),
    srcs = glob([
        "trt_lib/libnvinfer.so*",
        "trt_lib/libnvinfer_plugin.so*",
        "trt_lib/libnvonnxparser.so*",
    ]),
    # 只暴露必要的 include 路径，不再干扰 C++ 标准库
    includes = [
        "trt_include",
        "trt_include/aarch64-linux-gnu",
    ],
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
