load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

# ---------- shared helper: headers ----------
cc_library(
    name = "opencv_headers",
    hdrs = glob(["include/opencv4/opencv2/**/*.h"]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
)

# ---------- core ----------
cc_import(
    name = "core_so",
    shared_library = "lib/libopencv_core.so.4.4",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "core",
    deps = [
      ":opencv_headers",
      ":core_so"
    ],
    linkstatic = False,
    visibility = ["//visibility:public"],
)

# ---------- imgproc ----------
cc_import(
    name = "imgproc_so",
    shared_library = "lib/libopencv_imgproc.so.4.4",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "imgproc",
    deps = [
      ":opencv_headers",
      ":core",
      ":imgproc_so"
    ],
    linkstatic = False,
    visibility = ["//visibility:public"],
)

# ---------- imgcodecs ----------
cc_import(
    name = "imgcodecs_so",
    shared_library = "lib/libopencv_imgcodecs.so.4.4",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "imgcodecs",
    deps = [
      ":opencv_headers",
      ":core",
      ":imgproc",
      ":imgcodecs_so"
    ],
    linkstatic = False,
    visibility = ["//visibility:public"],
)

# ---------- highgui ----------
cc_import(
    name = "highgui_so",
    shared_library = "lib/libopencv_highgui.so.4.4",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "highgui",
    deps = [
        ":opencv_headers",
        ":core",
        ":imgproc",
        ":highgui_so",
    ],
    linkstatic = False,
    visibility = ["//visibility:public"],
)
