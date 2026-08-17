load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(
    default_visibility = ["//visibility:public"],
)

exports_files([
    "CPPLINT.cfg",
    "tox.ini",
])

install(
    name = "install",
    deps = if_gpu(
        [
            "//modules/perception:install",
            "//modules/planning:install",
        ],
        [
            "//tools:install",
            "//modules/calibration:install",
            "//modules/canbus:install",
            "//modules/common:install",
            "//modules/control:install",
            "//modules/planning:install",
            "//modules/dreamview:install",
            "//modules/drivers:install",
            "//modules/guardian:install",
            "//modules/localization:install",
            "//modules/map:install",
            "//modules/monitor:install",
            "//modules/prediction:install",
            "//modules/routing:install",
            "//modules/storytelling:install",
            "//modules/task_manager:install",
            "//modules/transform:install",
            "//scripts:install",
            "//third_party/ad_rss_lib:install",
            "//third_party/ipopt:install",
            "//third_party/opengl:install",
            "//third_party/adolc:install",
            "//third_party/npp:install",
            "//third_party/tf2:install",
            "//third_party/localization_msf:install",
            "//third_party/rtklib:install",
        ],
    ),
)

install_src_files(
    name = "install_src",
    deps = if_gpu(
        [
            "//modules/perception:install_src",
            "//modules/planning:install_src",
        ],
        [
            "//tools:install_src",
            "//modules/common:install_src",
            "//modules/control:install_src",
            "//modules/dreamview:install_src",
            "//modules/map:install_src",
            "//modules/monitor:install_src",
            "//modules/planning:install_src",
            "//modules/routing:install_src",
            "//modules/task_manager:install_src",
            "//modules/transform:install_src",
            "//modules/calibration:install_src",
            "//modules/canbus:install_src",
            "//modules/drivers:install_src",
            "//modules/guardian:install_src",
            "//modules/localization:install_src",
            "//modules/prediction:install_src",
            "//modules/storytelling:install_src",
            "//third_party/ipopt:install_src",
            "//third_party/opengl:install_src",
            "//third_party/adolc:install_src",
            "//third_party/npp:install_src",
            "//third_party/tf2:install_src",
            "//third_party/localization_msf:install_src",
            "//third_party/rtklib:install_src",
        ],
    ),
)
