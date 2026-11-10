load("@core//runtime:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_filegroup")

pkg_filegroup(
    name = "package_files",
    srcs = [
        "//docker/scripts:package_files",
        "//products/low_speed/conf:package_files",
        "//scripts:package_files",
        "@hardware.audio//:package_files",
        "@hardware.camera//:package_files",
        "@hardware.canbus//:package_files",
        "@hardware.gnss//:package_files",
        "@hardware.image_decompress//:package_files",
        "@hardware.lidar//:package_files",
        "@hardware.radar//conti_radar:package_files",
        "@hardware.vehicle//:package_files",
        "@hardware.video//:package_files",
        "@wheelos_common//tools:package_files",
        "@wheelos_control//modules/control:package_files",
        "@wheelos_hmi//modules/dreamview:package_files",
        "@wheelos_lidar_perception//modules/lidar_fusion:package_files",
        "@wheelos_localization//modules/localization:package_files",
        "@wheelos_localization//modules/ndt_localization:package_files",
        "@wheelos_localization//modules/rtk_localization:package_files",
        "@wheelos_mission//modules/mission:package_files",
        "@wheelos_planning//modules/local_planning:package_files",
        "@wheelos_planning//modules/map:package_files",
        "@wheelos_planning//modules/open_space_planning:package_files",
        "@wheelos_planning//modules/planning:package_files",
        "@wheelos_prediction//modules/prediction:package_files",
        "@wheelos_routing//modules/routing:package_files",
        "@wheelos_safety//modules/collision_guardian:package_files",
        "@wheelos_safety//modules/guardian:package_files",
        "@wheelos_safety//modules/monitor:package_files",
        "@wheelos_simulation//modules/simulation:package_files",
        "@wheelos_transform//modules/transform:package_files",
        "@wheelos_world_model//modules/storytelling:package_files",
        "@wheelos_world_model//modules/traffic_light:package_files",
        "@wheelos_world_model//modules/world_model:package_files",
    ] + select({
        "//tools/platform:is_jetson_orin": ["@hardware.camera_gst//:package_files"],
        "//conditions:default": [],
    }) + select({
        "@wheelos_common//tools/platform:use_gpu": [
            "@wheelos_camera_perception//modules/camera_semantic_segmentation:package_files",
            "@wheelos_lidar_perception//modules/lidar_object_detection:package_files",
            "@wheelos_lidar_perception//modules/lidar_semantic_segmentation:package_files",
            "@wheelos_multimodal_perception//modules/bevfusion:package_files",
            "@wheelos_perception//modules/perception:package_files",
            "@wheelos_world_model//modules/lane:package_files",
        ],
        "//conditions:default": [],
    }),
    tags = ["manual"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
