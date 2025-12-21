load("@rules_cc//cc:defs.bzl", "cc_library", "cc_import")

# TODO: Starting with Bazel 8.3, cc_import supports strip_include_prefix.
# At that point we can collapse the current two-stage setup (headers cc_library + cc_import .so)
# into one cc_import rule using:
#   strip_include_prefix = "include"
# or
#   includes = ["include"]
# to expose headers properly and simplify maintenance.


# 1. Header files
cc_library(
  name = "osqp_headers",
  hdrs = glob(["include/osqp/**/*.h"]),
  includes = ["include"],
  visibility = ["//visibility:public"],
)

# 2. Shared library
cc_import(
  name = "osqp_so",
  shared_library = "lib/libosqp.so",    # Relative to new_local_repository.path
  visibility = ["//visibility:public"],
)

# 3. Aggregate target
cc_library(
  name = "osqp",
  deps = [
    ":osqp_headers",
    ":osqp_so"
  ],
  visibility = ["//visibility:public"],
)
