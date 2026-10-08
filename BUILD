load("//tools/install:install.bzl", "install")
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
            "//modules/global_config:install",
            "//modules/control:install",
            "//modules/planning:install",
            "//modules/dreamview:install",
            "//modules/drivers:install",
            "//modules/guardian:install",
            "//modules/ndt_localization:install",
            "//modules/map:install",
            "//modules/monitor:install",
            "//modules/prediction:install",
            "//modules/routing:install",
            "//modules/rtk_localization:install",
            "//modules/storytelling:install",
            "//modules/task_manager:install",
            "//modules/transform:install",
            "//scripts:install",
            "//third_party/ad_rss_lib:install",
            "//third_party/ipopt:install",
            "//third_party/opengl:install",
            "//third_party/adolc:install",
            "//third_party/tf2:install",
            "//third_party/rtklib:install",
        ],
    ),
)
