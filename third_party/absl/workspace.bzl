"""Loads the absl library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # TODO(zero): need delete, use http_archive instead
    # native.new_local_repository(
    #     name = "com_google_absl",
    #     build_file = clean_dep("//third_party/absl:absl.BUILD"),
    #     path = "/opt/apollo/absl/",
    # )
    pass
