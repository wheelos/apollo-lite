load("@rules_python//python:defs.bzl", "py_library")
load("@rules_python//python:packaging.bzl", "py_package", "py_wheel", "py_wheel_dist")

package(default_visibility = ["//visibility:public"])


py_wheel(
  name = "cyber_wheel",
  distribution = "pycyber",
  version = "0.1.0",
  python_tag = "py3",
  deps = [":cyber_pkg"],
)

py_wheel_dist(
  name = "cyber_dist",
  wheel = ":cyber_wheel",
  out = "dist",
)

py_package(
  name = "cyber_pkg",
  packages = ["pycyber"],
  deps = [
    ":cyber",
    ":cyber_time",
    ":cyber_timer",
    ":parameter",
    ":record",
  ],
)

py_library(
    name = "cyber_time",
    srcs = ["cyber_time.py"],
    data = [
        "//cyber/python/internal:_cyber_time_wrapper.so",
        "//cyber/python/internal:_cyber_wrapper.so",
        "@fastrtps",
    ],
)

py_library(
    name = "cyber_timer",
    srcs = ["cyber_timer.py"],
    data = [
        "//cyber/python/internal:_cyber_timer_wrapper.so",
        "@fastrtps",
    ],
)

py_library(
    name = "cyber",
    srcs = ["cyber.py"],
    data = [
        "//cyber/python/internal:_cyber_wrapper.so",
        "@fastrtps",
    ],
)

py_library(
    name = "parameter",
    srcs = ["parameter.py"],
    data = [
        "//cyber/python/internal:_cyber_parameter_wrapper.so",
        "@fastrtps",
    ],
)

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//cyber/python/internal:_cyber_record_wrapper.so",
        "@fastrtps",
    ],
)
