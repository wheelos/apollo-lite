"""Loads the libtorch_gpu library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# The environment distinguishes between cpu version and gpu version,
# rather than having two versions in one container


def repo_cpu():
    native.new_local_repository(
        name = "libtorch_cpu",
        build_file = clean_dep("//third_party/libtorch:libtorch_cpu.BUILD"),
        path = "/usr/local/libtorch/include",
    )

def repo_gpu():
    native.new_local_repository(
        name = "libtorch_gpu",
        build_file = clean_dep("//third_party/libtorch:libtorch_gpu.BUILD"),
        path = "/usr/local/libtorch/include",
    )
