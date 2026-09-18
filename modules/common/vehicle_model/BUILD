load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    targets = [
        ":vehicle_model",
    ],
    visibility = ["//visibility:public"],
    deps = ["vehicle_model_data"],
)

install(
    name = "vehicle_model_data",
    data = [
        ":vehicle_model_config_data",
        ":vehicle_model_test_data",
    ],
    data_dest = "common/addition_data/vehicle_model",
)

filegroup(
    name = "vehicle_model_test_data",
    srcs = glob([
        "testdata/*",
    ]),
)

cc_library(
    name = "vehicle_model",
    srcs = [
        "vehicle_model.cc",
    ],
    hdrs = ["vehicle_model.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@core//cyber",
    ],
)

cc_test(
    name = "vehicle_model_test",
    size = "small",
    srcs = ["vehicle_model_test.cc"],
    data = [
        ":test_data",
        ":vehicle_model_config_data",
    ],
    deps = [
        ":vehicle_model",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:gtest_exit_main",
        "@core//cyber",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/*.txt",
    ]),
)

filegroup(
    name = "vehicle_model_config_data",
    srcs = ["conf/vehicle_model_config.pb.txt"],
)

cpplint()
