# Apollo external dependencies that can be loaded in WORKSPACE files.
load("//third_party/absl:workspace.bzl", absl = "repo")
load("//third_party/adolc:workspace.bzl", adolc = "repo")
load("//third_party/adv_plat:workspace.bzl", adv_plat = "repo")
load("//third_party/ad_rss_lib:workspace.bzl", ad_rss_lib = "repo")
load("//third_party/atlas:workspace.bzl", atlas = "repo")
load("//third_party/benchmark:workspace.bzl", benchmark = "repo")
load("//third_party/boost:workspace.bzl", boost = "repo")
load("//third_party/caddn_infer_op:workspace.bzl", caddn_infer_op = "repo")
load("//third_party/centerpoint_infer_op:workspace.bzl", centerpoint_infer_op = "repo")
load("//third_party/civetweb:workspace.bzl", civetweb = "repo")
load("//third_party/cpplint:workspace.bzl", cpplint = "repo")
load("//third_party/eigen3:workspace.bzl", eigen = "repo")
load("//third_party/ffmpeg:workspace.bzl", ffmpeg = "repo")
load("//third_party/fftw3:workspace.bzl", fftw3 = "repo")
load("//third_party/fastrtps:workspace.bzl", fastrtps = "repo")
load("//third_party/glog:workspace.bzl", glog = "repo")
load("//third_party/gtest:workspace.bzl", gtest = "repo")
load("//third_party/gflags:workspace.bzl", gflags = "repo")
load("//third_party/ipopt:workspace.bzl", ipopt = "repo")
load("//third_party/libtorch:workspace.bzl", libtorch_cpu = "repo_cpu", libtorch_gpu = "repo_gpu")
load("//third_party/ncurses5:workspace.bzl", ncurses5 = "repo")
load("//third_party/nlohmann_json:workspace.bzl", nlohmann_json = "repo")
load("//third_party/npp:workspace.bzl", npp = "repo")
load("//third_party/opencv:workspace.bzl", opencv = "repo")
load("//third_party/opengl:workspace.bzl", opengl = "repo")
load("//third_party/openh264:workspace.bzl", openh264 = "repo")
load("//third_party/osqp:workspace.bzl", osqp = "repo")
load("//third_party/paddleinference:workspace.bzl", paddleinference = "repo")
load("//third_party/portaudio:workspace.bzl", portaudio = "repo")
load("//third_party/proj:workspace.bzl", proj = "repo")
load("//third_party/protobuf:workspace.bzl", protobuf = "repo")
load("//third_party/qt5:workspace.bzl", qt5 = "repo")
load("//third_party/sqlite3:workspace.bzl", sqlite3 = "repo")
load("//third_party/tinyxml2:workspace.bzl", tinyxml2 = "repo")
load("//third_party/uuid:workspace.bzl", uuid = "repo")
load("//third_party/yaml_cpp:workspace.bzl", yaml_cpp = "repo")
load("//third_party/sse2neon:workspace.bzl", sse2neon = "repo")
load("//third_party/localization_msf:workspace.bzl", localization_msf = "repo")
# load("//third_party/glew:workspace.bzl", glew = "repo")

load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
load("//third_party/py:python_configure.bzl", "python_configure")
load("//third_party/tensorrt:tensorrt_configure.bzl", "tensorrt_configure")
load("//third_party/vtk:vtk_configure.bzl", "vtk_configure")
load("//third_party/pcl:pcl_configure.bzl", "pcl_configure")
load("//third_party/vanjee_driver:workspace.bzl", vanjee_driver = "repo")
load("//third_party/rs_driver:workspace.bzl", rs_driver = "repo")

def initialize_third_party():
    """ Load third party repositories.  See above load() statements. """
    # TODO(zero): Use bzlmod instead, when test ok will delete later
    # absl()
    adolc()
    adv_plat()
    ad_rss_lib()
    atlas()
    # benchmark()
    # boost()
    caddn_infer_op()
    centerpoint_infer_op()
    # cpplint()
    # civetweb()
    # eigen()
    fastrtps()
    ffmpeg()
    fftw3()
    # gflags()
    # glog()
    # gtest()
    ipopt()
    libtorch_cpu()
    libtorch_gpu()
    # ncurses5()
    # nlohmann_json()
    localization_msf()
    npp()
    opencv()
    opengl()
    openh264()
    osqp()
    paddleinference()
    # portaudio()
    proj()
    # protobuf()
    qt5()
    # sqlite3()
    # tinyxml2()
    # uuid()
    # yaml_cpp()
    # sse2neon()
    vanjee_driver()
    rs_driver()

# Define all external repositories required by
def apollo_repositories():
    # TODO(All): update to bazelmod, use custom rules instead of macros
    cuda_configure(name = "local_config_cuda")

    tensorrt_configure(name = "local_config_tensorrt")
    python_configure(name = "local_config_python")
    vtk_configure(name = "local_config_vtk")
    # pcl_configure(name = "local_config_pcl")

    initialize_third_party()
