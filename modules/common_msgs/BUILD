load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("@rules_python//python:packaging.bzl", "py_package", "py_wheel", "py_wheel_dist")
load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "common-msgs",
    data = [
        ":common-msgs.BUILD",
    ],
    deps = [
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "common-msgs/include",
    data = [
        "//modules/common_msgs/audio_msgs:audio_common_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_cc_proto",
        "//modules/common_msgs/basic_msgs:direction_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_id_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/dreamview_msgs:chart_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/common_msgs/guardian_msgs:guardian_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/map_msgs:map_clear_area_cc_proto",
        "//modules/common_msgs/map_msgs:map_crosswalk_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/map_msgs:map_junction_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/map_msgs:map_overlap_cc_proto",
        "//modules/common_msgs/map_msgs:map_parking_space_cc_proto",
        "//modules/common_msgs/map_msgs:map_pnc_junction_cc_proto",
        "//modules/common_msgs/map_msgs:map_road_cc_proto",
        "//modules/common_msgs/map_msgs:map_rsu_cc_proto",
        "//modules/common_msgs/map_msgs:map_signal_cc_proto",
        "//modules/common_msgs/map_msgs:map_speed_bump_cc_proto",
        "//modules/common_msgs/map_msgs:map_speed_control_cc_proto",
	      "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/common_msgs/map_msgs:map_stop_sign_cc_proto",
        "//modules/common_msgs/map_msgs:map_yield_sign_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_camera_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_lane_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:scenario_type_cc_proto",
        "//modules/common_msgs/planning_msgs:sl_boundary_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_point_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "//modules/common_msgs/routing_msgs:poi_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:delphi_esr_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/sensor_msgs:mobileye_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:racobit_radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
    ]
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "common-msgs/src",
    filter = "*",
)


py_package(
    name = "common_msgs_py_package",
    packages = ["whl_msgs"],
    deps = [
        ":common_msgs_py_pb2",
    ],
)


py_wheel(
    name = "whl_msgs_wheel",
    distribution = "whl_msgs",
    version = "0.1.0",
    python_tag = "py3",
    deps = [":common_msgs_py_package"],
)

py_wheel_dist(
    name = "whl_msgs_dist",
    wheel = ":whl_msgs_wheel",
    out = "dist",
)

py_library(
    name = "common_msgs_py_pb2",
    deps = [
        "//modules/common_msgs/audio_msgs:audio_common_py_pb2",
        "//modules/common_msgs/audio_msgs:audio_event_py_pb2",
        "//modules/common_msgs/audio_msgs:audio_py_pb2",
        "//modules/common_msgs/chassis_msgs:chassis_detail_py_pb2",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/config_msgs:vehicle_config_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/control_msgs:input_debug_py_pb2",
        "//modules/common_msgs/control_msgs:pad_msg_py_pb2",
        "//modules/common_msgs/dreamview_msgs:chart_py_pb2",
        "//modules/common_msgs/dreamview_msgs:hmi_status_py_pb2",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_py_pb2",
        "//modules/common_msgs/guardian_msgs:guardian_py_pb2",
        "//modules/common_msgs/localization_msgs:gps_py_pb2",
        "//modules/common_msgs/localization_msgs:imu_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_status_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/localization_msgs:pose_py_pb2",
        "//modules/common_msgs/map_msgs:map_clear_area_py_pb2",
        "//modules/common_msgs/map_msgs:map_crosswalk_py_pb2",
        "//modules/common_msgs/map_msgs:map_geometry_py_pb2",
        "//modules/common_msgs/map_msgs:map_id_py_pb2",
        "//modules/common_msgs/map_msgs:map_junction_py_pb2",
        "//modules/common_msgs/map_msgs:map_lane_py_pb2",
        "//modules/common_msgs/map_msgs:map_overlap_py_pb2",
        "//modules/common_msgs/map_msgs:map_parking_space_py_pb2",
        "//modules/common_msgs/map_msgs:map_pnc_junction_py_pb2",
        "//modules/common_msgs/map_msgs:map_road_py_pb2",
        "//modules/common_msgs/map_msgs:map_rsu_py_pb2",
        "//modules/common_msgs/map_msgs:map_signal_py_pb2",
        "//modules/common_msgs/map_msgs:map_speed_bump_py_pb2",
        "//modules/common_msgs/map_msgs:map_speed_control_py_pb2",
        "//modules/common_msgs/map_msgs:map_stop_sign_py_pb2",
        "//modules/common_msgs/map_msgs:map_yield_sign_py_pb2",
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/monitor_msgs:monitor_log_py_pb2",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_py_pb2",
        "//modules/common_msgs/monitor_msgs:system_status_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_camera_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_lane_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_py_pb2",
        "//modules/common_msgs/planning_msgs:decision_py_pb2",
        "//modules/common_msgs/planning_msgs:navigation_py_pb2",
        "//modules/common_msgs/planning_msgs:pad_msg_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_internal_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/common_msgs/planning_msgs:scenario_type_py_pb2",
        "//modules/common_msgs/planning_msgs:sl_boundary_py_pb2",
        "//modules/common_msgs/prediction_msgs:feature_py_pb2",
        "//modules/common_msgs/prediction_msgs:lane_graph_py_pb2",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_py_pb2",
        "//modules/common_msgs/prediction_msgs:prediction_point_py_pb2",
        "//modules/common_msgs/prediction_msgs:scenario_py_pb2",
        "//modules/common_msgs/routing_msgs:poi_py_pb2",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/common_msgs/sensor_msgs:conti_radar_py_pb2",
        "//modules/common_msgs/sensor_msgs:delphi_esr_py_pb2",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_py_pb2",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_py_pb2",
        "//modules/common_msgs/sensor_msgs:gnss_py_pb2",
        "//modules/common_msgs/sensor_msgs:heading_py_pb2",
        "//modules/common_msgs/sensor_msgs:imu_py_pb2",
        "//modules/common_msgs/sensor_msgs:ins_py_pb2",
        "//modules/common_msgs/sensor_msgs:mobileye_py_pb2",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/sensor_msgs:racobit_radar_py_pb2",
        "//modules/common_msgs/sensor_msgs:radar_py_pb2",
        "//modules/common_msgs/sensor_msgs:sensor_image_py_pb2",
        "//modules/common_msgs/sensor_msgs:smartereye_py_pb2",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_py_pb2",
        "//modules/common_msgs/simulation_msgs:agent_py_pb2",
        "//modules/common_msgs/simulation_msgs:grading_condition_py_pb2",
        "//modules/common_msgs/simulation_msgs:grading_metric_py_pb2",
        "//modules/common_msgs/simulation_msgs:scenario_py_pb2",
        "//modules/common_msgs/storytelling_msgs:story_py_pb2",
        "//modules/common_msgs/task_manager_msgs:task_manager_py_pb2",
        "//modules/common_msgs/transform_msgs:transform_py_pb2",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_py_pb2",
        "//modules/common_msgs/basic_msgs:direction_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
        "//modules/common_msgs/basic_msgs:error_code_py_pb2",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
        "//modules/common_msgs/basic_msgs:pnc_point_py_pb2",
        "//modules/common_msgs/basic_msgs:vehicle_signal_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_event_py_pb2",
        "//modules/common_msgs/basic_msgs:vehicle_id_py_pb2",
    ],
)
