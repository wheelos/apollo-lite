"""Whole-module production roots; internal algorithm selection stays runtime-owned."""

MODULE_TARGETS = {
    "bevfusion": "//modules/bevfusion:libbevfusion_component_plugin.so",
    "camera_semantic_segmentation": "//modules/camera_semantic_segmentation:libcamera_semantic_segmentation_component_plugin.so",
    "collision_guardian": "//modules/collision_guardian:libcollision_guardian_component.so",
    "control": "//modules/control:libcontrol_component.so",
    "guardian": "//modules/guardian:libguardian_component.so",
    "lane": "//modules/lane:lane_component_plugin",
    "lidar_fusion": "//modules/lidar_fusion:liblidar_fusion_component.so",
    "lidar_object_detection": "//modules/lidar_object_detection:liblidar_object_detection_component.so",
    "lidar_semantic_segmentation": "//modules/lidar_semantic_segmentation:liblidar_semantic_segmentation_component_plugin.so",
    "local_planning": "//modules/local_planning:lane_follow_planner",
    "mission": "//modules/mission:libmission_component.so",
    "ndt_localization": "//modules/ndt_localization:libndt_localization_component.so",
    "open_space_planning": "//modules/open_space_planning:libopen_space_planning_component.so",
    "planning": "//modules/planning:libplanning_component.so",
    "prediction": "//modules/prediction:libprediction_component.so",
    "routing": "//modules/routing:routing_service",
    "rtk_localization": "//modules/rtk_localization:librtk_localization_component.so",
    "storytelling": "//modules/storytelling:libstorytelling_component.so",
    "traffic_light": "//modules/traffic_light:libtraffic_light_component.so",
    "transform": "@wheelos_transform//modules/transform:libstatic_transform_component.so",
}

def selected_targets():
    targets = []
    for name, target in MODULE_TARGETS.items():
        targets += select({
            ":with_" + name: [target],
            "//conditions:default": [],
        })
    return targets

def _module_list_impl(ctx):
    modules = ctx.build_setting_value
    unknown = [name for name in modules if name not in MODULE_TARGETS]
    if unknown:
        fail("Unknown algorithm modules: {}. Available: {}".format(
            ", ".join(unknown),
            ", ".join(sorted(MODULE_TARGETS)),
        ))
    if not modules:
        fail("Select algorithm modules with --//tools/modules:include=<module,...> or --config=algo_rtk_open_space")
    return []

module_list = rule(
    implementation = _module_list_impl,
    build_setting = config.string_list(flag = True),
)
