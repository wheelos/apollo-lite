# Apollo external dependencies that can be loaded in WORKSPACE files.
load("//third_party/adolc:workspace.bzl", adolc = "repo")
load("//third_party/adv_plat:workspace.bzl", adv_plat = "repo")
load("//third_party/ad_rss_lib:workspace.bzl", ad_rss_lib = "repo")
load("//third_party/atlas:workspace.bzl", atlas = "repo")
load("//third_party/caddn_infer_op:workspace.bzl", caddn_infer_op = "repo")
load("//third_party/centerpoint_infer_op:workspace.bzl", centerpoint_infer_op = "repo")
load("//third_party/ffmpeg:workspace.bzl", ffmpeg = "repo")
load("//third_party/fftw3:workspace.bzl", fftw3 = "repo")
load("//third_party/ipopt:workspace.bzl", ipopt = "repo")
load("//third_party/libtorch:workspace.bzl", libtorch_cpu = "repo_cpu", libtorch_gpu = "repo_gpu")
load("//third_party/npp:workspace.bzl", npp = "repo")
load("//third_party/opencv:workspace.bzl", opencv = "repo")
load("//third_party/opengl:workspace.bzl", opengl = "repo")
load("//third_party/openh264:workspace.bzl", openh264 = "repo")
load("//third_party/osqp:workspace.bzl", osqp = "repo")
load("//third_party/paddleinference:workspace.bzl", paddleinference = "repo")
load("//third_party/proj:workspace.bzl", proj = "repo")
load("//third_party/qt5:workspace.bzl", qt5 = "repo")
load("//third_party/localization_msf:workspace.bzl", localization_msf = "repo")

load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
load("//third_party/py:python_configure.bzl", "python_configure")
load("//third_party/tensorrt:tensorrt_configure.bzl", "tensorrt_configure")
load("//third_party/vanjee_driver:workspace.bzl", vanjee_driver = "repo")
load("//third_party/rs_driver:workspace.bzl", rs_driver = "repo")

def initialize_third_party():
    """ Load third party repositories.  See above load() statements. """
    # TODO(zero): Use bzlmod instead, when test ok will delete later
    adolc()
    adv_plat()
    ad_rss_lib()
    atlas()
    caddn_infer_op()
    centerpoint_infer_op()
    ffmpeg()
    fftw3()
    ipopt()
    libtorch_cpu()
    libtorch_gpu()
    localization_msf()
    npp()
    opencv()
    opengl()
    openh264()
    osqp()
    paddleinference()
    proj()
    qt5()
    vanjee_driver()
    rs_driver()

# Define all external repositories required by
def apollo_repositories():
    # TODO(All): update to bazelmod, use custom rules instead of macros
    cuda_configure(name = "local_config_cuda")

    tensorrt_configure(name = "local_config_tensorrt")
    python_configure(name = "local_config_python")

    initialize_third_party()
