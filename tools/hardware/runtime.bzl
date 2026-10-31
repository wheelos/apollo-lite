"""Materialize one runtime layout for source development and binary packaging."""

load("@rules_pkg//pkg:providers.bzl", "PackageFilegroupInfo", "PackageFilesInfo")
load("@rules_python//python:defs.bzl", "py_binary")
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
    entries.append([ctx.file.cyber_launch.path, "bin/cyber_launch.py"])
    inputs.append(ctx.file.cyber_launch)
    entries.append([ctx.file.runtime_entry.path, "bin/runtime_entry.py"])
    inputs.append(ctx.file.runtime_entry)
    for name in ["cyber_recorder", "cyber_monitor"]:
        file = getattr(ctx.executable, name)
        entries.append([file.path, "bin/" + name])
        inputs.append(file)
    for file in ctx.files.cyber_conf:
        entries.append([file.path, _repository_path(file).removeprefix("cyber/")])
        inputs.append(file)
    for file in ctx.files.metadata:
        entries.append([file.path, "hardware/" + file.basename])
        inputs.append(file)

    libraries = []
    for target in ctx.attr.components + [
        ctx.attr.mainboard,
        ctx.attr.cyber_recorder,
        ctx.attr.cyber_monitor,
    ]:
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
    ctx.actions.write(spec, json.encode({
        "files": entries,
        "libraries": libraries,
        "default_dag": ctx.attr.default_dag,
    }))
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
        "default_dag": attr.string(),
        "runtime_entry": attr.label(
            default = "//tools/hardware:runtime_entry.py",
            allow_single_file = True,
        ),
        "cyber_launch": attr.label(
            default = "@core//cyber/tools/cyber_launch:cyber_launch.py",
            allow_single_file = True,
        ),
        "mainboard": attr.label(
            default = "@core//cyber/mainboard:mainboard",
            executable = True,
            cfg = "target",
        ),
        "cyber_recorder": attr.label(
            default = "@core//cyber/tools/cyber_recorder:cyber_recorder",
            executable = True,
            cfg = "target",
        ),
        "cyber_monitor": attr.label(
            default = "@core//cyber/tools/cyber_monitor:cyber_monitor",
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

def runtime_run(name, runtime):
    py_binary(
        name = name,
        srcs = [Label("//tools/hardware:run_runtime.py")],
        main = Label("//tools/hardware:run_runtime.py"),
        data = [runtime],
        args = ["$(rlocationpath " + runtime + ")"],
        deps = ["@rules_python//python/runfiles"],
        precompile = "disabled",
        legacy_create_init = 0,
    )

def _runtime_package_impl(ctx):
    executable_dirs = ["bin", "lib", "bazel-bin"]
    data_dirs = ["modules", "conf", "hardware"]
    trees = {
        directory: ctx.actions.declare_directory(ctx.label.name + "/" + directory)
        for directory in executable_dirs + data_dirs
    }
    ctx.actions.run(
        executable = ctx.executable._materializer,
        arguments = [
            "--package",
            ctx.file.runtime.path,
            trees["bin"].dirname,
        ],
        inputs = [ctx.file.runtime],
        outputs = trees.values(),
        mnemonic = "RuntimePackageMapping",
    )
    return [
        DefaultInfo(files = depset(trees.values())),
        PackageFilegroupInfo(
            pkg_files = [
                (PackageFilesInfo(
                    dest_src_map = {directory: trees[directory] for directory in directories},
                    attributes = {"mode": mode},
                ), ctx.label)
                for directories, mode in [(executable_dirs, "0755"), (data_dirs, "0644")]
            ],
            pkg_dirs = [],
            pkg_symlinks = [],
        ),
    ]

runtime_package = rule(
    implementation = _runtime_package_impl,
    attrs = {
        "runtime": attr.label(mandatory = True, allow_single_file = True),
        "_materializer": attr.label(
            default = "//tools/hardware:materialize_runtime",
            executable = True,
            cfg = "exec",
        ),
    },
)
