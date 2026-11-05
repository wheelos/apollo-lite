load("@apollo_products//tools/products:defs.bzl", "runtime_product")
load("@core//runtime:runtime.bzl", "runtime_module")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")

package(default_visibility = ["//visibility:private"])

cc_binary(
    name = "libalpha.so",
    testonly = True,
    srcs = [
        "alpha.cc",
        "fixtures/runtime_fixture.h",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "@core//cyber",
        "@core//cyber/common:resource_manager",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
    ],
)

cc_binary(
    name = "libbeta.so",
    testonly = True,
    srcs = [
        "beta.cc",
        "fixtures/runtime_fixture.h",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "@core//cyber",
        "@core//cyber/common:resource_manager",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
    ],
)

py_binary(
    name = "diagnose",
    testonly = True,
    srcs = ["diagnose.py"],
    deps = ["@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_py_pb2"],
)

runtime_module(
    name = "alpha",
    testonly = True,
    components = [":libalpha.so"],
    executables = [":diagnose"],
    files = {
        "alpha.conf": "conf/alpha/component.conf",
        "alpha.dag": "dag/alpha/component.dag",
        "model.bin": "assets/models/alpha/fixture/model.bin",
    },
    module_name = "alpha",
)

runtime_module(
    name = "beta",
    testonly = True,
    components = [":libbeta.so"],
    files = {
        "beta.conf": "conf/beta/component.conf",
        "beta.dag": "dag/beta/component.dag",
    },
    module_name = "beta",
)

ASSETS = {
    "models/alpha/fixture/model.bin": "3147ff706091c46ed758da951a053ad025d4929782bee3591d163cc527715ecf",
}

runtime_product(
    name = "strict",
    testonly = True,
    default_dag = "dag/alpha/component.dag",
    modules = [":alpha"],
    required_assets = ASSETS,
    required_configs = ["conf/alpha/component.conf"],
)

runtime_product(
    name = "single",
    testonly = True,
    default_dag = "dag/alpha/component.dag",
    modules = [":alpha"],
    required_assets = ASSETS,
)

runtime_product(
    name = "combined",
    testonly = True,
    default_launch = "launch/autonomy/combined.launch",
    files = {"combined.launch": "launch/autonomy/combined.launch"},
    modules = [
        ":alpha",
        ":beta",
    ],
    required_assets = ASSETS,
)

py_test(
    name = "integration",
    size = "medium",
    timeout = "long",
    srcs = ["integration_test.py"],
    args = [
        "$(rlocationpath :single)",
        "$(rlocationpath :combined)",
        "$(rlocationpath :combined_package)",
        "$(rlocationpath :combined_install)",
        "$(rlocationpath :strict)",
    ],
    data = [
        ":combined",
        ":combined_install",
        ":combined_package",
        ":single",
        ":strict",
    ],
    main = "integration_test.py",
    tags = ["exclusive"],
    deps = ["@rules_python//python/runfiles"],
)
