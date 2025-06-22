load("@rules_python//python:packaging.bzl", "py_wheel")

package(
    default_visibility = ["//visibility:public"],
)

py_wheel(
    name = "pycyber",
    distribution = "pycyber",
    version = "1.0.0",
    author = "wheelos",
    author_email = "wheelos@wheelos.cn",
    homepage = "https://github.com/wheelos/apollo",
    license = "Apache-2.0",
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
        "//cyber/python/cyber_py3:cyber_timer",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:parameter",
        "//cyber/python/cyber_py3:record",
    ],
    data_files = {
        "//cyber/python/internal:_cyber_wrapper.so": "platlib/cyber/python/internal/_cyber_wrapper.so",
        "//cyber/python/internal:_cyber_time_wrapper.so": "platlib/cyber/python/internal/_cyber_time_wrapper.so",
        "//cyber/python/internal:_cyber_timer_wrapper.so": "platlib/cyber/python/internal/_cyber_timer_wrapper.so",
        "//cyber/python/internal:_cyber_parameter_wrapper.so": "platlib/cyber/python/internal/_cyber_parameter_wrapper.so",
        "//cyber/python/internal:_cyber_record_wrapper.so": "platlib/cyber/python/internal/_cyber_record_wrapper.so"
    },
    requires = [
        "protobuf>=3.12.0",
    ],
    classifiers = [
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
)
