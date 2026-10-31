load(":defs.bzl", "MODULE_TARGETS", "module_list", "selected_targets")

package(default_visibility = ["//visibility:public"])

module_list(
    name = "include",
    build_setting_default = [],
)

[
    config_setting(
        name = "with_" + name,
        flag_values = {":include": name},
    )
    for name in MODULE_TARGETS
]

filegroup(
    name = "algorithms",
    srcs = [":include"] + selected_targets(),
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
