load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_python//python:defs.bzl", "py_test")
load("//tools/hardware:runtime.bzl", "runtime_layout", "runtime_package")
load(":module_test.bzl", "module_tests")

package(default_visibility = ["//visibility:private"])

exports_files(["models/model_fixture.ts"])

module_tests()

config_setting(
    name = "fake_can",
    flag_values = {
        "@hardware.build//tools/hardware:canbus": "fake",
        "@hardware.build//tools/hardware:vehicle": "devkit",
    },
)

cc_binary(
    name = "record_probe",
    testonly = True,
    srcs = ["record_probe.cc"],
    deps = [
        "@core//cyber/record:record_reader",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

runtime_layout(
    name = "runtime",
    testonly = True,
    cyber_conf = "conf/cyber.pb.conf",
    files = {"models/model_fixture.ts": "models/example/model.engine"},
    modules = [
        "@vehicle_infra//products/vehicle:module",
        "@sensor_infra//tests/sensors:module",
    ],
)

runtime_package(
    name = "package_files",
    testonly = True,
    runtime = ":runtime",
)

pkg_tar(
    name = "archive",
    testonly = True,
    srcs = [":package_files"],
    extension = "tar.gz",
)

pkg_install(
    name = "installer",
    testonly = True,
    srcs = [":package_files"],
)

py_test(
    name = "integration",
    size = "medium",
    timeout = "moderate",
    srcs = ["integration_test.py"],
    args = [
        "$(rlocationpath :runtime)",
        "$(rlocationpath :record_probe)",
        "$(rlocationpath :archive)",
        "$(rlocationpath :installer)",
    ],
    data = [
        ":archive",
        ":installer",
        ":record_probe",
        ":runtime",
    ],
    main = "integration_test.py",
    tags = [
        "exclusive",
        "manual",
    ],
    target_compatible_with = select({
        ":fake_can": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = ["@rules_python//python/runfiles"],
)
