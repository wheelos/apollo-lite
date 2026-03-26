# Apollo external dependencies that can be loaded in WORKSPACE files.
load("//third_party/ad_rss_lib:workspace.bzl", ad_rss_lib = "repo")
load("//third_party/adolc:workspace.bzl", adolc = "repo")
load("//third_party/atlas:workspace.bzl", atlas = "repo")
load("//third_party/fftw3:workspace.bzl", fftw3 = "repo")
load("//third_party/ipopt:workspace.bzl", ipopt = "repo")
load("//third_party/localization_msf:workspace.bzl", localization_msf = "repo")
load("//third_party/npp:workspace.bzl", npp = "repo")
load("//third_party/opengl:workspace.bzl", opengl = "repo")
load("//third_party/openh264:workspace.bzl", openh264 = "repo")

def initialize_third_party():
    """ Load third party repositories.  See above load() statements. """

    # TODO(zero): Use bzlmod instead, when test ok will delete later
    adolc()
    ad_rss_lib()
    atlas()
    fftw3()
    ipopt()
    localization_msf()
    npp()
    opengl()
    openh264()

# Define all external repositories required by
def apollo_repositories():
    initialize_third_party()
