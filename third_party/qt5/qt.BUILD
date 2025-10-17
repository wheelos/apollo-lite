load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "qt_core",
    hdrs = glob(["*"]),
    copts = [
        "-I.",
    ],
    includes = [
        "QtCore",
    ],
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-L/lib/aarch64-linux-gnu", 
            "-Wl,-rpath,/lib/aarch64-linux-gnu",
        ],
        "//conditions:default": [
            "-L/usr/local/qt5/lib",
            "-Wl,-rpath,/usr/local/qt5/lib",
        ],
    }) + [
        "-lQt5Core",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt_widgets",
    hdrs = glob(["*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtWidgets"],
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-L/lib/aarch64-linux-gnu", 
            "-Wl,-rpath,/lib/aarch64-linux-gnu",
        ],
        "//conditions:default": [
            "-L/usr/local/qt5/lib",
            "-Wl,-rpath,/usr/local/qt5/lib",
        ],
    }) + [
        "-lQt5Widgets",
    ],
    visibility = ["//visibility:public"],
    deps = [":qt_core"],
)

cc_library(
    name = "qt_gui",
    hdrs = glob(["*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtGui"],
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-L/lib/aarch64-linux-gnu", 
            "-Wl,-rpath,/lib/aarch64-linux-gnu",
        ],
        "//conditions:default": [
            "-L/usr/local/qt5/lib",
            "-Wl,-rpath,/usr/local/qt5/lib",
        ],
    }) + [
        "-lQt5Gui",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":qt_core",
        ":qt_widgets",
    ],
)

cc_library(
    name = "qt_opengl",
    hdrs = glob(["*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtOpenGL"],
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-L/lib/aarch64-linux-gnu", 
            "-Wl,-rpath,/lib/aarch64-linux-gnu",
        ],
        "//conditions:default": [
            "-L/usr/local/qt5/lib",
            "-Wl,-rpath,/usr/local/qt5/lib",
        ],
    }) + [
        "-lQt5OpenGL",
        "-lGL",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":qt_core",
        ":qt_gui",
        ":qt_widgets",
        #"@opengl",
    ],
)

