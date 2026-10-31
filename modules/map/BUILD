load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(
    default_visibility = ["//visibility:public"],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**/*",
    ]),
)

pkg_files(
    name = "map_testdata_data",
    srcs = [":testdata"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "map/addition_data",
    strip_prefix = strip_prefix.from_root("modules/map"),
)

pkg_filegroup(
    name = "map_testdata",
    srcs = [":map_testdata_data"],
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":map_testdata",
        ":pb_map",
        "//modules/map/pnc_map:package_files",
        "//modules/map/tools:package_files",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

pkg_files(
    name = "pb_map_data",
    srcs = [
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_clear_area_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_crosswalk_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_geometry_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_id_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_junction_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_lane_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_overlap_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_parking_space_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_pnc_junction_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_road_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_rsu_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_signal_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_speed_bump_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_stop_sign_py_pb2",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_yield_sign_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:navigation_py_pb2",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "map",
    strip_prefix = strip_prefix.from_pkg(),
)

pkg_filegroup(
    name = "pb_map",
    srcs = [":pb_map_data"],
)

pkg_filegroup(
    name = "source_files",
    srcs = [
        ":package_map_hdrs",
        ":package_map_src",
    ],
)

pkg_install(
    name = "source_dir",
    srcs = [":source_files"],
    tags = ["manual"],
)

pkg_files(
    name = "package_map_src_data",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ) + [
        "//modules/map/pnc_map:exported_source_files",
        "//modules/map/tools:exported_source_files",
        "//modules/map/tools/map_datachecker/client:exported_source_files",
        "//modules/map/tools/map_datachecker/proto:exported_source_files",
        "//modules/map/tools/map_datachecker/server:exported_source_files",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "map/src",
    strip_prefix = strip_prefix.from_root("modules/map"),
)

pkg_filegroup(
    name = "package_map_src",
    srcs = [":package_map_src_data"],
)

pkg_files(
    name = "package_map_hdrs_data",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ) + [
        "//modules/map/pnc_map:exported_source_headers",
        "//modules/map/tools/map_datachecker/client:exported_source_headers",
        "//modules/map/tools/map_datachecker/server:exported_source_headers",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "map/include",
    strip_prefix = strip_prefix.from_root("modules/map"),
)

pkg_filegroup(
    name = "package_map_hdrs",
    srcs = [":package_map_hdrs_data"],
)
