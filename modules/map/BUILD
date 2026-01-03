load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**/*",
    ]),
)

install(
    name = "map_testdata",
    data_dest = "map/addition_data",
    data = [":testdata"],
)

install(
    name = "install",
    data_dest = "map",
    data = [
        ":map.BUILD",
    ],
    deps = [
        ":pb_map",
        # "//modules/map/data:install",
        "//modules/map/tools:install",
        "//modules/map/hdmap/adapter:install",
        "//modules/map/hdmap:install",
        "//modules/map/pnc_map:install",
        ":map_testdata"
    ],
)

install_files(
    name = "pb_map",
    dest = "map",
    files = [
        "//modules/common_msgs/map_msgs:map_clear_area_py_pb2",
        "//modules/common_msgs/map_msgs:map_crosswalk_py_pb2",
        "//modules/common_msgs/map_msgs:map_geometry_py_pb2",
        "//modules/common_msgs/map_msgs:map_id_py_pb2",
        "//modules/common_msgs/map_msgs:map_junction_py_pb2",
        "//modules/common_msgs/map_msgs:map_lane_py_pb2",
        "//modules/common_msgs/map_msgs:map_overlap_py_pb2",
        "//modules/common_msgs/map_msgs:map_parking_space_py_pb2",
        "//modules/common_msgs/map_msgs:map_pnc_junction_py_pb2",
        "//modules/common_msgs/map_msgs:map_proto",
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/common_msgs/map_msgs:map_road_py_pb2",
        "//modules/common_msgs/map_msgs:map_rsu_py_pb2",
        "//modules/common_msgs/map_msgs:map_signal_py_pb2",
        "//modules/common_msgs/map_msgs:map_speed_bump_py_pb2",
        "//modules/common_msgs/map_msgs:map_stop_sign_py_pb2",
        "//modules/common_msgs/map_msgs:map_yield_sign_py_pb2",
        "//modules/common_msgs/planning_msgs:navigation_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_map_src",
        ":install_map_hdrs"
    ],
)

install_src_files(
    name = "install_map_src",
    src_dir = ["."],
    dest = "map/src",
    filter = "*",
)

install_src_files(
    name = "install_map_hdrs",
    src_dir = ["."],
    dest = "map/include",
    filter = "*.h",
)
