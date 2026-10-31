load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion",
    srcs = [
        "core/alignment/input_validation.cc",
        "core/alignment/input_validation.h",
        "core/alignment/sweep_identity.cc",
        "core/association/build_candidates.cc",
        "core/association/build_candidates.h",
        "core/association/detection_association.cc",
        "core/association/detection_association.h",
        "core/association/prior_association.cc",
        "core/association/prior_association.h",
        "core/evidence/build_evidence.cc",
        "core/evidence/build_evidence.h",
        "core/evidence/ground_plane.cc",
        "core/evidence/ground_plane.h",
        "core/evidence/point_geometry.cc",
        "core/evidence/point_geometry.h",
        "core/runtime/single_frame_fusion.cc",
        "core/spatial/connected_voxels.cc",
        "core/spatial/connected_voxels.h",
        "core/spatial/occupancy.cc",
        "core/spatial/occupancy.h",
        "publication/functional_scene.cc",
    ],
    hdrs = [
        "core/alignment/sweep_identity.h",
        "core/fusion.h",
        "core/runtime/single_frame_fusion.h",
        "publication/functional_scene.h",
    ],
    deps = [
        ":candidates",
        ":spatial",
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_cc_proto",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "candidates",
    srcs = [
        "core/association/candidate_components.cc",
        "core/association/candidate_geometry.cc",
        "core/association/candidate_geometry.h",
        "core/association/candidate_relations.cc",
        "core/association/relation_support.h",
    ],
    hdrs = [
        "core/association/candidates.h",
        "core/candidates.h",
    ],
    deps = [
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "spatial",
    srcs = [
        "core/spatial/grid_queries.h",
        "core/spatial/point_views.cc",
        "core/spatial/space_cells.cc",
        "core/spatial/spatial.cc",
        "core/spatial/spatial_stages.h",
        "core/spatial/visibility.cc",
    ],
    hdrs = [
        "core/spatial.h",
        "core/spatial/spatial.h",
    ],
    deps = [
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "terrain_surface",
    srcs = [
        "core/terrain/plane_fit.cc",
        "core/terrain/plane_fit.h",
        "core/terrain/surface.cc",
        "core/terrain/surface_config.cc",
        "core/terrain/surface_internal.h",
        "core/terrain/surface_layers.cc",
        "core/terrain/surface_measurements.cc",
        "core/terrain/surface_status.cc",
    ],
    hdrs = [
        "core/terrain/surface.h",
        "core/terrain_surface.h",
    ],
    deps = ["@eigen"],
)

cc_library(
    name = "terrain_traversability",
    srcs = ["core/terrain/traversability.cc"],
    hdrs = [
        "core/terrain/traversability.h",
        "core/terrain_traversability.h",
    ],
    deps = [":terrain_surface"],
)

cc_library(
    name = "temporal",
    srcs = [
        "core/tracking/history.cc",
        "core/tracking/temporal.cc",
        "core/tracking/temporal_internal.h",
        "core/tracking/tracker_adapter.cc",
    ],
    hdrs = [
        "core/temporal.h",
        "core/tracking/temporal.h",
    ],
    deps = [
        ":fusion",
        ":spatial",
        "@eigen",
        "@wheelos_common//tracking/core:tracking",
        "@wheelos_common//tracking/estimation:predictor",
    ],
)

cc_library(
    name = "frame_buffer",
    srcs = ["core/alignment/frame_buffer.cc"],
    hdrs = [
        "core/alignment/frame_buffer.h",
        "core/frame_buffer.h",
    ],
    deps = [":fusion"],
)

cc_library(
    name = "pod_publication",
    srcs = ["publication/pod.cc"],
    hdrs = ["publication/pod.h"],
    deps = [
        "//modules/lidar_fusion/proto:lidar_fusion_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
    ],
)

cc_library(
    name = "lidar_fusion_component_lib",
    srcs = ["component/lidar_fusion_component.cc"],
    hdrs = ["component/lidar_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"lidar_fusion\\"'],
    deps = [
        ":frame_buffer",
        ":pod_publication",
        ":temporal",
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_binary(
    name = "terrain_diagnostics",
    srcs = ["tools/terrain_diagnostics.cc"],
    deps = [
        ":terrain_surface",
        ":terrain_traversability",
    ],
)

cc_binary(
    name = "replay_cached",
    srcs = ["tools/replay_cached.cc"],
    deps = [
        ":fusion",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "publication_probe",
    srcs = ["tools/publication_probe.cc"],
    tags = ["manual"],
    deps = [
        ":pod_publication",
        "@core//cyber",
    ],
)

cc_binary(
    name = "replay_fusion",
    srcs = ["tools/replay_fusion.cc"],
    deps = [
        ":temporal",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "replay",
    srcs = ["tools/replay.cc"],
    target_compatible_with = select({
        "//tools/platform:use_gpu": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":fusion",
        ":temporal",
        "//modules/lidar_object_detection:pointpillars_input",
        "//modules/lidar_object_detection:pointpillars_runtime_api",
        "//modules/lidar_semantic_segmentation:rangeret",
        "//modules/lidar_semantic_segmentation:tensorrt_executor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "liblidar_fusion_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_fusion_component_lib"],
)

cc_test(
    name = "fusion_test",
    size = "small",
    srcs = [
        "core/association/candidate_construction_test.cc",
        "core/association/detection_association_test.cc",
        "core/evidence/ground_evidence_test.cc",
        "core/runtime/fusion_test.cc",
        "core/runtime/fusion_test_support.h",
        "core/spatial/occupancy_test.cc",
        "publication/functional_scene_test.cc",
    ],
    deps = [
        ":fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "temporal_test",
    size = "small",
    srcs = ["core/tracking/temporal_test.cc"],
    deps = [
        ":temporal",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "candidates_test",
    size = "small",
    srcs = ["core/association/candidates_test.cc"],
    deps = [
        ":candidates",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spatial_test",
    size = "small",
    srcs = ["core/spatial/spatial_test.cc"],
    deps = [
        ":spatial",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "terrain_test",
    size = "small",
    srcs = ["core/terrain/terrain_test.cc"],
    deps = [
        ":terrain_surface",
        ":terrain_traversability",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_buffer_test",
    size = "small",
    srcs = ["core/alignment/frame_buffer_test.cc"],
    deps = [
        ":frame_buffer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pod_publication_test",
    size = "small",
    srcs = ["publication/pod_test.cc"],
    deps = [
        ":pod_publication",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "component_test",
    size = "small",
    srcs = ["component/lidar_fusion_component_test.cc"],
    data = ["conf/lidar_fusion.pb.txt"],
    tags = ["manual"],
    deps = [
        ":lidar_fusion_component_lib",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
    ],
)

filegroup(
    name = "runtime_config",
    srcs = glob([
        "conf/**",
        "dag/**",
    ]),
)

pkg_files(
    name = "package_files_data",
    srcs = [":runtime_config"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lidar_fusion",
    strip_prefix = strip_prefix.from_root("modules/lidar_fusion"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":liblidar_fusion_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "lidar_fusion/lib",
    strip_prefix = strip_prefix.from_root("modules/lidar_fusion"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
