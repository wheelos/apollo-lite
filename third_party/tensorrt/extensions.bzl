load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def _tensorrt_configure_impl(ctx):
    # 1. Acquisition path: takes precedence over environment variables,
    # defaults to the standard path from fallback to Jetpack.
    trt_root = ctx.os.environ.get("TENSORRT_INSTALL_PATH", "/usr")

    # 2. Dynamically generate BUILD file
    build_content = """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt",
    hdrs = glob([
        "include/aarch64-linux-gnu/NvInfer*.h",
        "include/aarch64-linux-gnu/NvUtils.h",
        "include/NvInfer*.h",
        "include/NvUtils.h",
    ]),
    srcs = glob([
        # Jetpack / Debian 布局
        "lib/aarch64-linux-gnu/libnvinfer.so*",
        "lib/aarch64-linux-gnu/libnvinfer_plugin.so*",
        "lib/aarch64-linux-gnu/libnvonnxparser.so*",
        # 兼容 tarball 安装方式
        "lib/libnvinfer.so*",
        "lib/libnvinfer_plugin.so*",
        "lib/libnvonnxparser.so*",
    ]),
    includes = [
        "include",
        "include/aarch64-linux-gnu"
    ],
    defines = ["TF_NEED_TENSORRT=1"],
    linkstatic = 0, # 强制动态链接系统库
)
"""

    # 3. Create a repository and automatically process symlinks.
    ctx.file("BUILD", build_content)

configure_tag = tag_class(attrs = {"path": attr.string()})

tensorrt_configure = module_extension(
    implementation = _tensorrt_configure_impl,
    tag_classes = {"configure": configure_tag},
    environ = ["TENSORRT_INSTALL_PATH"],
)
