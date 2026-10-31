load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qt_core",
    hdrs = glob(["QtCore/**"]),
    includes = [
        ".",
        "QtCore",
    ],
    deps = [":qt_core_shared"],
)

cc_library(
    name = "qt_widgets",
    hdrs = glob(["QtWidgets/**"]),
    includes = [
        ".",
        "QtWidgets",
    ],
    deps = [
        ":qt_core",
        ":qt_widgets_shared",
    ],
)

cc_library(
    name = "qt_gui",
    hdrs = glob(["QtGui/**"]),
    includes = [
        ".",
        "QtGui",
    ],
    deps = [
        ":qt_core",
        ":qt_gui_shared",
        ":qt_widgets",
    ],
)

cc_library(
    name = "qt_opengl",
    hdrs = glob(["QtOpenGL/**"]),
    includes = [
        ".",
        "QtOpenGL",
    ],
    linkopts = ["-lGL"],
    deps = [
        ":qt_core",
        ":qt_gui",
        ":qt_opengl_shared",
        ":qt_widgets",
    ],
)

cc_import(
    name = "qt_core_shared",
    shared_library = "lib/libQt5Core.so",
)

cc_import(
    name = "qt_widgets_shared",
    shared_library = "lib/libQt5Widgets.so",
)

cc_import(
    name = "qt_gui_shared",
    shared_library = "lib/libQt5Gui.so",
)

cc_import(
    name = "qt_opengl_shared",
    shared_library = "lib/libQt5OpenGL.so",
)

exports_files([
    "bin/moc",
    "bin/uic",
    "bin/rcc",
])
