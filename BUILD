load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_filegroup")

exports_files(
    ["CPPLINT.cfg"],
    visibility = ["//visibility:public"],
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        "//docker/scripts:package_files",
        "//modules/bevfusion:package_files",
        "//modules/camera_semantic_segmentation:package_files",
        "//modules/collision_guardian:package_files",
        "//modules/control:package_files",
        "//modules/dreamview:package_files",
        "//modules/global_config:package_files",
        "//modules/guardian:package_files",
        "//modules/lane:package_files",
        "//modules/lidar_fusion:package_files",
        "//modules/lidar_object_detection:package_files",
        "//modules/lidar_semantic_segmentation:package_files",
        "//modules/localization:package_files",
        "//modules/map:package_files",
        "//modules/monitor:package_files",
        "//modules/ndt_localization:package_files",
        "//modules/perception:package_files",
        "//modules/planning:package_files",
        "//modules/prediction:package_files",
        "//modules/routing:package_files",
        "//modules/rtk_localization:package_files",
        "//modules/simulation:package_files",
        "//modules/storytelling:package_files",
        "//scripts:package_files",
        "@hardware.audio//:package_files",
        "@hardware.camera//:package_files",
        "@hardware.camera_gst//:package_files",
        "@hardware.canbus//:package_files",
        "@hardware.gnss//:package_files",
        "@hardware.image_decompress//:package_files",
        "@hardware.lidar//:package_files",
        "@hardware.radar//conti_radar:package_files",
        "@hardware.vehicle//:package_files",
        "@hardware.video//:package_files",
        "@wheelos_transform//modules/transform:package_files",
    ],
    tags = ["manual"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
