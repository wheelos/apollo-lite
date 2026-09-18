load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":3rd-ad-rss-lib.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-ad-rss-lib",
    deps = [
        "@ad_rss_lib//:install",
    ],
)

install_src_files(
    name = "install_src",
    dest = "3rd-ad-rss-lib/src",
    filter = "*",
    src_dir = ["."],
)
