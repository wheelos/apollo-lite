"""Compatibility exports; core owns the reusable system assembly rules."""

load(
    "@core//runtime:runtime.bzl",
    _RuntimeModuleInfo = "RuntimeModuleInfo",
    _required_runtime_input = "required_runtime_input",
    _runtime_layout = "runtime_layout",
    _runtime_module = "runtime_module",
    _runtime_package = "runtime_package",
    _runtime_resources = "runtime_resources",
    _runtime_run = "runtime_run",
)

RuntimeModuleInfo = _RuntimeModuleInfo
required_runtime_input = _required_runtime_input
runtime_layout = _runtime_layout
runtime_module = _runtime_module
runtime_package = _runtime_package
runtime_run = _runtime_run
runtime_resources = _runtime_resources
