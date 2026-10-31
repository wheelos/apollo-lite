"""cquery formatter: describe analyzed choices without executing build actions."""

def format(target):
    selection = providers(target).get("//tools/hardware:defs.bzl%HardwareSelectionInfo")
    if selection:
        return "Hardware (effective): " + str(selection.choices)
    options = build_options(target)
    copts = options.get("//command_line_option:copt", [])
    lines = [
        "Root: " + str(target.label),
        "Platform: " + str(options.get("//command_line_option:platforms")),
        "Mode: " + str(options.get("//command_line_option:compilation_mode")),
        "Compute: " + ("GPU" if "-DUSE_GPU=1" in copts else "CPU") + " (C++ compile configuration)",
    ]
    modules = options.get("//tools/modules:include")
    if modules:
        lines.append("Modules: " + ", ".join(modules))
    lines.append("Outputs:")
    lines.extend(["  " + f.path for f in target.files.to_list()])
    return "\n".join(lines)
