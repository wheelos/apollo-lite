load("@rules_cc//cc:defs.bzl", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    deps = [
        "//modules/drivers/radar/conti_radar:install",
        "//modules/drivers/radar/racobit_radar:install",
        "//modules/drivers/radar/ultrasonic_radar:install",
    ],
)

cc_test(
    name = "radar_protocol_test",
    size = "small",
    srcs = ["radar_protocol_test.cc"],
    deps = [
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
        "//modules/drivers/radar/nano_radar/protocol:drivers_nano_radar_protocol",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
        "//modules/drivers/radar/udas_ultrasonic_radar/protocol:canbus_udas_ultrasonic_radar_protocol",
        "//modules/drivers/radar/yg_radar/protocol:canbus_yg_radar_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
