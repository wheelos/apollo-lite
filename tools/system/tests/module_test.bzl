"""Module contribution ownership is enforced during analysis."""

load("@bazel_skylib//lib:unittest.bzl", "analysistest", "asserts")
load("@core//runtime:runtime.bzl", "RuntimeModuleInfo", "runtime_module", "runtime_resources")

def _module_impl(ctx):
    env = analysistest.begin(ctx)
    module = analysistest.target_under_test(env)[RuntimeModuleInfo]
    asserts.equals(env, "example", module.name)
    asserts.equals(env, ["conf/example/example.conf"], [
        destination
        for _, destination in module.entries
    ])
    return analysistest.end(env)

def _ownership_impl(ctx):
    env = analysistest.begin(ctx)
    asserts.expect_failure(env, "shared Cyber configuration is owned by runtime_layout")
    return analysistest.end(env)

def _contribution_impl(ctx):
    env = analysistest.begin(ctx)
    target = analysistest.target_under_test(env)
    module = target[RuntimeModuleInfo]
    asserts.true(env, len(module.entries) > 0)
    for _, destination in module.entries:
        allowed_prefixes = [
            "lib/components/" + module.name + "/",
            "conf/" + module.name + "/",
            "dag/" + module.name + "/",
            "launch/" + module.name + "/",
            "models/" + module.name + "/",
            "share/" + module.name + "/",
            "bin/modules/" + module.name + "/",
        ]
        asserts.true(env, any([
            destination.startswith(prefix)
            for prefix in allowed_prefixes
        ]))
    for file in target[DefaultInfo].files.to_list():
        asserts.false(env, file.basename in ["mainboard", "cyber_recorder", "cyber_monitor", "cyber.pb.conf"])
    return analysistest.end(env)

module_test = analysistest.make(_module_impl)
ownership_test = analysistest.make(_ownership_impl, expect_failure = True)
contribution_test = analysistest.make(_contribution_impl)

def module_tests():
    runtime_module(
        name = "example_module",
        testonly = True,
        module_name = "example",
        files = {"example.conf": "conf/example/example.conf"},
    )
    module_test(name = "module_test", target_under_test = ":example_module")
    runtime_resources(
        name = "example_model_resources",
        files = ["models/model_fixture.ts"],
        module_name = "example",
        namespace = "example",
    )
    runtime_module(
        name = "model_example_module",
        testonly = True,
        module_name = "example",
        resources = [":example_model_resources"],
    )
    contribution_test(name = "model_module_test", target_under_test = ":model_example_module")
    runtime_module(
        name = "invalid_module",
        testonly = True,
        module_name = "example",
        files = {"example.conf": "conf/cyber.pb.conf"},
        tags = ["manual"],
    )
    ownership_test(name = "ownership_test", target_under_test = ":invalid_module")
    contribution_test(name = "vehicle_contribution_test", target_under_test = "@vehicle_infra//products/vehicle:module")
    contribution_test(name = "sensor_contribution_test", target_under_test = "@sensor_infra//tests/sensors:module")
