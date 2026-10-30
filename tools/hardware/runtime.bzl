"""Materialize one runtime layout for source development and binary packaging."""

load("@rules_pkg//pkg:providers.bzl", "PackageFilesInfo")
load(":runtime_paths.bzl", "deployment_path")

def _repository_path(file):
    path = file.short_path
    if path.startswith("../"):
        path = path.split("/", 2)[2]
    return path

def _runtime_layout_impl(ctx):
    entries = []
    inputs = []
    for file in ctx.files.components:
        entries.append([file.path, "bazel-bin/" + deployment_path(file)])
        inputs.append(file)
    for file in ctx.files.runtime_data:
        entries.append([file.path, deployment_path(file)])
        inputs.append(file)
    for file in ctx.files.generated_configs:
        destination = {
            "gnss_conf.pb.txt": "modules/drivers/gnss/conf/gnss_conf.pb.txt",
            "canbus_conf.pb.txt": "modules/canbus/conf/canbus_conf.pb.txt",
        }[file.basename]
        entries.append([file.path, destination])
        inputs.append(file)
    mainboard = ctx.executable.mainboard
    entries.append([mainboard.path, "bin/mainboard"])
    inputs.append(mainboard)
    for file in ctx.files.cyber_conf:
        entries.append([file.path, _repository_path(file).removeprefix("cyber/")])
        inputs.append(file)
    for file in ctx.files.metadata:
        entries.append([file.path, "hardware/" + file.basename])
        inputs.append(file)

    libraries = []
    for target in ctx.attr.components + [ctx.attr.mainboard]:
        info = target[DefaultInfo]
        for file in depset(transitive = [
            info.default_runfiles.files,
            info.data_runfiles.files,
        ]).to_list():
            if file.basename.endswith(".so") or ".so." in file.basename:
                libraries.append([file.path, "lib/" + file.basename])
                inputs.append(file)

    tree = ctx.actions.declare_directory(ctx.label.name)
    spec = ctx.actions.declare_file(ctx.label.name + ".layout.json")
    ctx.actions.write(spec, json.encode({"files": entries, "libraries": libraries}))
    ctx.actions.run(
        executable = ctx.executable._materializer,
        arguments = [spec.path, tree.path],
        inputs = depset(inputs + [spec]),
        outputs = [tree],
        mnemonic = "HardwareRuntimeLayout",
    )
    return [
        DefaultInfo(files = depset([tree])),
        PackageFilesInfo(
            dest_src_map = {".": tree},
            attributes = {},
        ),
    ]

runtime_layout = rule(
    implementation = _runtime_layout_impl,
    attrs = {
        "components": attr.label_list(allow_files = True),
        "runtime_data": attr.label_list(allow_files = True),
        "generated_configs": attr.label_list(allow_files = True),
        "metadata": attr.label_list(allow_files = True),
        "mainboard": attr.label(
            default = "@core//cyber/mainboard:mainboard",
            executable = True,
            cfg = "target",
        ),
        "cyber_conf": attr.label(
            default = "@core//cyber:cyber_conf",
            allow_files = True,
        ),
        "_materializer": attr.label(
            default = "//tools/hardware:materialize_runtime",
            executable = True,
            cfg = "exec",
        ),
    },
)
