load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":calibrated_vehicles",
        ":calibration.BUILD",
    ],
    data_dest = "calibration",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_calibration_hdrs",
        ":install_calibration_src",
    ],
)

install_src_files(
    name = "install_calibration_src",
    dest = "calibration/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_calibration_hdrs",
    dest = "calibration/include",
    filter = "*.h",
    src_dir = ["."],
)

filegroup(
    name = "calibrated_vehicles",
    srcs = glob([
        "data/**",
    ]),
)
