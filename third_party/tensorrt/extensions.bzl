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

def _get_system_include_path(repository_ctx):
    """Get system include path based on CPU architecture."""
    if _is_arm64(repository_ctx):
        return "/usr/include/aarch64-linux-gnu"
    elif _is_x86_64(repository_ctx):
        return "/usr/include/x86_64-linux-gnu"
    return "/usr/include"

def _get_system_library_path(repository_ctx):
    """Get system library path based on CPU architecture."""
    if _is_arm64(repository_ctx):
        return "/usr/lib/aarch64-linux-gnu"
    elif _is_x86_64(repository_ctx):
        return "/usr/lib/x86_64-linux-gnu"
    return "/usr/lib"

def _parse_version_number(content, macro_name):
    """Parse version number from content like '#define NV_TENSORRT_MAJOR 10'."""
    # Find the macro definition
    marker = "#define " + macro_name + " "
    idx = content.find(marker)
    if idx == -1:
        return None
    # Extract the number after the macro
    start = idx + len(marker)
    # Find the end of the number
    num_str = ""
    for i in range(start, len(content)):
        c = content[i]
        if c.isdigit():
            num_str += c
        else:
            break
    if num_str:
        return int(num_str)
    return None

def _get_tensorrt_version(repository_ctx, include_path):
    """Detect TensorRT version by reading NvInferVersion.h."""
    version_file = repository_ctx.path(include_path + "/NvInferVersion.h")
    if version_file.exists:
        result = repository_ctx.execute(["cat", str(version_file)])
        if result.return_code == 0:
            content = result.stdout
            major = _parse_version_number(content, "NV_TENSORRT_MAJOR")
            if major != None:
                return major
    # Default to version 10 if cannot determine
    return 10

def _setup_tensorrt8(repository_ctx, system_include_path, system_library_path):
    """Setup TensorRT 8.x headers, libraries and BUILD file."""
    include_files = [
        "NvInfer.h",
        "NvInferPlugin.h",
        "NvInferPluginUtils.h",
        "NvInferVersion.h",
        "NvOnnxConfig.h",
        "NvOnnxParser.h",
        "NvCaffeParser.h",
        "NvUffParser.h"
    ]

    library_files = [
        "libnvinfer.so",
        "libnvinfer_plugin.so",
        "libnvonnxparser.so",
        "libnvparsers.so"
    ]

    for file in include_files:
        _symlink_file_if_exists(repository_ctx, system_include_path, file, "tensorrt/include")

    for file in library_files:
        _symlink_file_if_exists(repository_ctx, system_library_path, file, "tensorrt/lib")

    build_content = """package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt",
    hdrs = glob(["tensorrt/include/*.h"]),
    srcs = glob([
        "tensorrt/lib/libnvinfer.so",
        "tensorrt/lib/libnvinfer_plugin.so",
        "tensorrt/lib/libnvonnxparser.so",
        "tensorrt/lib/libnvparsers.so",
    ]),
    includes = ["tensorrt/include"],
    linkstatic = 0,
)
"""
    repository_ctx.file("BUILD", build_content)

def _setup_tensorrt10(repository_ctx, system_include_path, system_library_path):
    """Setup TensorRT 10.x headers, libraries and BUILD file."""
    include_files = [
        "NvInfer.h",
        "NvInferImpl.h",
        "NvInferLegacyDims.h",
        "NvInferPlugin.h",
        "NvInferPluginBase.h",
        "NvInferPluginUtils.h",
        "NvInferRuntime.h",
        "NvInferRuntimeBase.h",
        "NvInferRuntimeCommon.h",
        "NvInferRuntimePlugin.h",
        "NvInferVersion.h",
        "NvOnnxConfig.h",
        "NvOnnxParser.h"
    ]

    library_files = [
        "libnvinfer.so",
        "libnvinfer_plugin.so",
        "libnvonnxparser.so"
    ]

    for file in include_files:
        _symlink_file_if_exists(repository_ctx, system_include_path, file, "tensorrt/include")

    for file in library_files:
        _symlink_file_if_exists(repository_ctx, system_library_path, file, "tensorrt/lib")

    build_content = """package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt",
    hdrs = glob(["tensorrt/include/*.h"]),
    srcs = glob([
        "tensorrt/lib/libnvinfer.so",
        "tensorrt/lib/libnvinfer_plugin.so",
        "tensorrt/lib/libnvonnxparser.so",
    ]),
    includes = ["tensorrt/include"],
    linkstatic = 0,
)
"""
    repository_ctx.file("BUILD", build_content)

def _tensorrt_repository_impl(repository_ctx):
    system_include_path = _get_system_include_path(repository_ctx)
    system_library_path = _get_system_library_path(repository_ctx)

    # Detect TensorRT version
    trt_version = _get_tensorrt_version(repository_ctx, system_include_path)

    if trt_version <= 8:
        _setup_tensorrt8(repository_ctx, system_include_path, system_library_path)
    else:
        _setup_tensorrt10(repository_ctx, system_include_path, system_library_path)

tensorrt_repository = repository_rule(
    implementation = _tensorrt_repository_impl,
    attrs = {
        "path": attr.string(default = "/usr"),
    },
)

def _tensorrt_configure_impl(ctx):
    tensorrt_repository(
        name = "local_config_tensorrt",
        path = "/usr",
    )

tensorrt_configure = module_extension(implementation = _tensorrt_configure_impl)
