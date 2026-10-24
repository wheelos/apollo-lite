"""Local TensorRT SDK configuration for the supported development images."""

def _get_cpu_arch(repository_ctx):
    return repository_ctx.os.arch

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
        return repository_ctx.attr.path + "/include/aarch64-linux-gnu"
    elif _is_x86_64(repository_ctx):
        return repository_ctx.attr.path + "/include/x86_64-linux-gnu"
    fail("Unsupported TensorRT architecture: {}".format(_get_cpu_arch(repository_ctx)))

def _get_system_library_path(repository_ctx):
    """Get system library path based on CPU architecture."""
    if _is_arm64(repository_ctx):
        return repository_ctx.attr.path + "/lib/aarch64-linux-gnu"
    elif _is_x86_64(repository_ctx):
        return repository_ctx.attr.path + "/lib/x86_64-linux-gnu"
    fail("Unsupported TensorRT architecture: {}".format(_get_cpu_arch(repository_ctx)))

def _parse_version_number(content, macro_name):
    """Parse version number from content like '#define NV_TENSORRT_MAJOR 10'."""
    for line in content.splitlines():
        fields = [field for field in line.replace("\t", " ").split(" ") if field]
        if len(fields) >= 3 and fields[:2] == ["#define", macro_name] and fields[2].isdigit():
            return int(fields[2])
    return None

def _get_tensorrt_version(repository_ctx, include_path):
    """Detect TensorRT version by reading NvInferVersion.h."""
    version_file = repository_ctx.path(include_path + "/NvInferVersion.h")
    if version_file.exists:
        major = _parse_version_number(repository_ctx.read(version_file), "NV_TENSORRT_MAJOR")
        if major != None:
            return major
    fail("Cannot determine TensorRT version from {}. Install TensorRT development packages.".format(version_file))

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
        "NvUffParser.h",
    ]

    library_files = [
        "libnvinfer.so",
        "libnvinfer_plugin.so",
        "libnvonnxparser.so",
        "libnvparsers.so",
    ]

    for file in include_files:
        _symlink_file_if_exists(repository_ctx, system_include_path, file, "tensorrt/include")

    for file in library_files:
        _symlink_file_if_exists(repository_ctx, system_library_path, file, "tensorrt/lib")

    build_content = """load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

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
        "NvOnnxParser.h",
    ]

    library_files = [
        "libnvinfer.so",
        "libnvinfer_plugin.so",
        "libnvonnxparser.so",
    ]

    for file in include_files:
        _symlink_file_if_exists(repository_ctx, system_include_path, file, "tensorrt/include")

    for file in library_files:
        _symlink_file_if_exists(repository_ctx, system_library_path, file, "tensorrt/lib")

    build_content = """load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

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
    if not repository_ctx.path(system_include_path + "/NvInferVersion.h").exists:
        system_include_path = repository_ctx.attr.path + "/include"
    if not repository_ctx.path(system_library_path + "/libnvinfer.so").exists:
        system_library_path = repository_ctx.attr.path + "/lib64"
    if not repository_ctx.path(system_library_path + "/libnvinfer.so").exists:
        system_library_path = repository_ctx.attr.path + "/lib"
    for library in ["libnvinfer.so", "libnvinfer_plugin.so", "libnvonnxparser.so"]:
        if not repository_ctx.path(system_library_path + "/" + library).exists:
            fail("TensorRT library not found: {}/{}".format(system_library_path, library))

    # Detect TensorRT version
    trt_version = _get_tensorrt_version(repository_ctx, system_include_path)

    if trt_version == 8:
        _setup_tensorrt8(repository_ctx, system_include_path, system_library_path)
    elif trt_version == 10:
        _setup_tensorrt10(repository_ctx, system_include_path, system_library_path)
    else:
        fail("Unsupported TensorRT major version: {} (supported: 8, 10)".format(trt_version))

tensorrt_repository = repository_rule(
    implementation = _tensorrt_repository_impl,
    attrs = {
        "path": attr.string(default = "/usr"),
    },
    local = True,
    configure = True,
)

def _tensorrt_configure_impl(_ctx):
    tensorrt_repository(
        name = "local_config_tensorrt",
        path = "/usr",
    )

tensorrt_configure = module_extension(implementation = _tensorrt_configure_impl)
