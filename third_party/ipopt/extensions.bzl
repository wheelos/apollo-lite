"""Bzlmod extension for the host IPOPT installation."""

def _ipopt_repository_impl(repository_ctx):
    include_dir = repository_ctx.getenv(
        "IPOPT_INCLUDE_DIR",
        repository_ctx.attr.include_dir,
    )
    include_path = repository_ctx.path(include_dir)
    if not include_path.exists:
        fail("IPOPT include directory not found: {}".format(include_path))
    if not repository_ctx.path(include_dir + "/coin/IpTNLP.hpp").exists:
        fail("IPOPT header not found: {}/coin/IpTNLP.hpp".format(include_dir))

    repository_ctx.symlink(str(include_path), "include")
    repository_ctx.symlink(
        Label("//third_party/ipopt:BUILD"),
        "BUILD.bazel",
    )

ipopt_repository = repository_rule(
    implementation = _ipopt_repository_impl,
    attrs = {
        "include_dir": attr.string(default = "/usr/include"),
    },
    environ = ["IPOPT_INCLUDE_DIR"],
    local = True,
    configure = True,
)

def _ipopt_extension_impl(_ctx):
    ipopt_repository(name = "ipopt")

ipopt = module_extension(
    implementation = _ipopt_extension_impl,
)
