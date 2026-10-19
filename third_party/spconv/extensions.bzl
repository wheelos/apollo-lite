def _spconv_repository_impl(repository_ctx):
    library = repository_ctx.os.environ.get("APOLLO_NVIDIA_SPCONV_LIBRARY", "")
    if not library:
        fail("Set APOLLO_NVIDIA_SPCONV_LIBRARY to the compatible libspconv.so path and pass it with --repo_env.")

    library_path = repository_ctx.path(library)
    if not library_path.exists:
        fail("APOLLO_NVIDIA_SPCONV_LIBRARY does not exist: {}".format(library))

    repository_ctx.symlink(library_path, "libspconv.so")
    repository_ctx.file(
        "BUILD",
        """package(default_visibility = ["//visibility:public"])

cc_import(
    name = "spconv",
    shared_library = "libspconv.so",
)

filegroup(
    name = "spconv_shared_library",
    srcs = ["libspconv.so"],
)
""",
    )

spconv_repository = repository_rule(
    implementation = _spconv_repository_impl,
    environ = ["APOLLO_NVIDIA_SPCONV_LIBRARY"],
    local = True,
)
