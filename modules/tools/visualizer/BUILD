load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//third_party/qt5:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_visualizer",
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkopts = [
        "-pthread",
    ],
    linkstatic = True,
    deps = [
        ":visualizer_lib",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

# TODO(all): Extract the large library to thin pieces.
qt_cc_library(
    name = "visualizer_lib",
    srcs = [
        "abstract_camera.cc",
        "fixedaspectratiowidget.cc",
        "free_camera.cc",
        "grid.cc",
        "main.cc",
        "main_window.cc",
        "msg_dialog.cc",
        "plane.cc",
        "pointcloud.cc",
        "radarpoints.cc",
        "renderable_object.cc",
        "scene_camera_dialog.cc",
        "scene_viewer.cc",
        "target_camera.cc",
        "texture.cc",
        "treewidget.cc",
        "video_image_viewer.cc",
        "video_images_dialog.cc",
    ],
    hdrs = [
        "abstract_camera.h",
        "channel_reader.h",
        "fixedaspectratiowidget.h",
        "free_camera.h",
        "grid.h",
        "main_window.h",
        "msg_dialog.h",
        "plane.h",
        "pointcloud.h",
        "radarpoints.h",
        "renderable_object.h",
        "scene_camera_dialog.h",
        "scene_viewer.h",
        "target_camera.h",
        "texture.h",
        "treewidget.h",
        "video_image_viewer.h",
        "video_images_dialog.h",
    ],
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
    res = glob([
        "*.qrc",
    ]),
    res_srcs = glob([
        "images/**/*",
    ]) + glob([
        "shaders/**/*",
    ]),
    uis = glob([
        "uis/*.ui",
    ]),
    deps = [
        "@core//cyber",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:radar_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

# TODO(all): Disable linter temporarily as the generated ui files should be
# excluded from check. But we should also check the .h and .cc files, if they
# are extracted to their own cc_libraries. See the TODO above.
