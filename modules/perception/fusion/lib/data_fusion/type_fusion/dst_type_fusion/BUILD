load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_type_fusion",
    srcs = ["dst_type_fusion.cc"],
    hdrs = ["dst_type_fusion.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:base_init_options",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/common:camera_util",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/config_manager",
        "//modules/perception/pipeline/proto/plugin:dst_type_fusion_config_cc_proto",
        "@boost.format",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "dst_type_fusion_test",
    size = "small",
    srcs = ["dst_type_fusion_test.cc"],
    # Workaround for gold linker bug on aarch64: disable Cortex-A53 erratum 843419
    # fix to avoid internal error in update_erratum_insn/try_fix_erratum.
    #
    # The gold linker has a known bug when applying CPU erratum fixes that causes
    # an internal error: "internal error in update_erratum_insn, at ../../gold/aarch64.cc:1005"
    # or "internal error in try_fix_erratum". This bug affects binutils including version 2.38.
    #
    # References:
    # - Original bug report (2017): https://sourceware.org/bugzilla/show_bug.cgi?id=20765
    # - Recent report (2023): https://mail.gnu.org/archive/html/bug-binutils/2023-12/msg00181.html
    # - Stack Overflow discussion: https://stackoverflow.com/questions/77010106/what-is-the-meaning-behind-internal-error-in-update-erratum-insn-at-gold
    # - Patch discussion: https://sourceware.org/pipermail/binutils/2017-November/100818.html
    #
    # The -mno-fix-cortex-a53-843419 flag disables the erratum workaround in the linker.
    # This only affects this test binary; production code remains unaffected.
    linkopts = ["-mno-fix-cortex-a53-843419"],
    deps = [
        ":dst_type_fusion",
        "//modules/perception/base:base_type",
        "//modules/perception/base:frame",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
