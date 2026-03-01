variable "APOLLO_REPO" {
  default = "wheelos/apollo"
}

variable "IMAGE_TAG" {
  default = "local"
}

variable "GEOLOC" {
  default = "us"
}

variable "INSTALL_MODE" {
  default = "download"
}

variable "LOCAL_HTTP_ADDR" {
  default = "http://172.17.0.1:8388"
}

variable "UBUNTU_LTS" {
  default = "22.04"
}

variable "CUDA_X86" {
  default = "12.6.3"
}

variable "CUDNN_X86_MAJOR" {
  default = "9"
}

variable "TENSORRT_X86" {
  default = "10.3.0.30"
}

variable "CUDA_TOOLKIT_X86" {
  default = "12.6.68"
}

variable "CUDNN_X86_FULL" {
  default = "9.3.0.75"
}

variable "VPI_VERSION" {
  default = "3.2.4"
}

variable "VULKAN_VERSION" {
  default = "1.3.204"
}

variable "L4T_TAG" {
  default = "r36.4.0"
}

group "base" {
  targets = ["base-x86_64-cuda", "base-aarch64-cuda"]
}

group "dev" {
  targets = ["dev-x86_64-cuda", "dev-aarch64-cuda"]
}

group "all" {
  targets = [
    "base-x86_64-cuda",
    "dev-x86_64-cuda",
    "base-aarch64-cuda",
    "dev-aarch64-cuda",
  ]
}

target "_common" {
  context    = "."
  pull       = true
  no-cache   = false
  platforms  = ["linux/amd64"]
}

target "base-x86_64-cuda" {
  inherits = ["_common"]
  dockerfile = "base.x86_64.cuda.dockerfile"
  platforms = ["linux/amd64"]
  tags = [
    "${APOLLO_REPO}:cuda${CUDA_X86}-cudnn${CUDNN_X86_MAJOR}-trt10-devel-${UBUNTU_LTS}-x86_64-${IMAGE_TAG}",
    "${APOLLO_REPO}:base-x86_64-${UBUNTU_LTS}-${IMAGE_TAG}",
  ]
  args = {
    BASE_IMAGE = "nvidia/cuda:${CUDA_X86}-cudnn-devel-ubuntu${UBUNTU_LTS}"
    CUDA_TOOLKIT_VERSION = "${CUDA_TOOLKIT_X86}"
    CUDNN_VERSION = "${CUDNN_X86_FULL}"
    TENSORRT_VERSION = "${TENSORRT_X86}"
    VPI_VERSION = "${VPI_VERSION}"
    VULKAN_VERSION = "${VULKAN_VERSION}"
  }
}

target "dev-x86_64-cuda" {
  inherits = ["_common"]
  dockerfile = "dev.x86_64.cuda.dockerfile"
  platforms = ["linux/amd64"]
  tags = [
    "${APOLLO_REPO}:dev-x86_64-${UBUNTU_LTS}-${IMAGE_TAG}",
  ]
  args = {
    BASE_IMAGE = "${APOLLO_REPO}:cuda${CUDA_X86}-cudnn${CUDNN_X86_MAJOR}-trt10-devel-${UBUNTU_LTS}-x86_64-${IMAGE_TAG}"
    GEOLOC = "${GEOLOC}"
    INSTALL_MODE = "${INSTALL_MODE}"
    LOCAL_HTTP_ADDR = "${LOCAL_HTTP_ADDR}"
  }
}

target "base-aarch64-cuda" {
  inherits = ["_common"]
  dockerfile = "base.aarch64.cuda.dockerfile"
  platforms = ["linux/arm64"]
  tags = [
    "${APOLLO_REPO}:jetpack-${L4T_TAG}-base-aarch64-${IMAGE_TAG}",
    "${APOLLO_REPO}:base-aarch64-${UBUNTU_LTS}-${IMAGE_TAG}",
  ]
  args = {
    BASE_IMAGE = "nvcr.io/nvidia/l4t-jetpack:${L4T_TAG}"
  }
}

target "dev-aarch64-cuda" {
  inherits = ["_common"]
  dockerfile = "dev.aarch64.cuda.dockerfile"
  platforms = ["linux/arm64"]
  tags = [
    "${APOLLO_REPO}:dev-aarch64-${UBUNTU_LTS}-${IMAGE_TAG}",
  ]
  args = {
    BASE_IMAGE = "${APOLLO_REPO}:jetpack-${L4T_TAG}-base-aarch64-${IMAGE_TAG}"
    GEOLOC = "${GEOLOC}"
    INSTALL_MODE = "${INSTALL_MODE}"
    LOCAL_HTTP_ADDR = "${LOCAL_HTTP_ADDR}"
  }
}
