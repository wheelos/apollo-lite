variable "APOLLO_REPO" {
  default = ""
}

variable "RELEASE_VERSION" {
  default = ""
}

variable "CACHE_REGISTRY" {
  default = ""
}

variable "GEOLOC" {
  default = "cn"
}

variable "LOCAL_HTTP_ADDR" {
  default = ""
}

group "default" {
  targets = [
    "dev-amd64-cpu-u22",
    "dev-amd64-cuda-u22",
  ]
}

target "_dev-common" {
  context = "."
  dockerfile = "docker/build/dev.dockerfile"
  target = "dev"
  args = {
    APOLLO_DIST = "stable"
    GEOLOC = GEOLOC
    LOCAL_HTTP_ADDR = LOCAL_HTTP_ADDR
  }
  output = ["type=docker"]
}

target "dev-amd64-cpu-u22" {
  inherits = ["_dev-common"]
  args = {
    BASE_IMAGE = "ubuntu:22.04"
    BASE_VARIANT = "cpu-u22"
    GPU_SUPPORT = "0"
  }
  platforms = ["linux/amd64"]
  tags = RELEASE_VERSION == "" ? [format("%s:dev-x86_64-22.04-cpu", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"))] : [format("%s:%s-amd64-u22-cpu", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"), RELEASE_VERSION)]
  cache-from = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-amd64-cpu-u22"]
  cache-to = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-amd64-cpu-u22,mode=max"]
}

target "dev-amd64-cuda-u22" {
  inherits = ["_dev-common"]
  args = {
    BASE_IMAGE = "nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04"
    BASE_VARIANT = "cuda-u22"
    GPU_SUPPORT = "1"
  }
  platforms = ["linux/amd64"]
  tags = RELEASE_VERSION == "" ? [format("%s:dev-x86_64-22.04-gpu", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"))] : [format("%s:%s-amd64-u22-cuda12.8", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"), RELEASE_VERSION)]
  cache-from = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-amd64-cuda-u22"]
  cache-to = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-amd64-cuda-u22,mode=max"]
}

target "dev-orin-jp621-l4t3643" {
  inherits = ["_dev-common"]
  args = {
    BASE_IMAGE = "nvcr.io/nvidia/l4t-tensorrt:r10.3.0-devel"
    BASE_VARIANT = "orin-jp621-l4t3643"
    GPU_SUPPORT = "1"
  }
  platforms = ["linux/arm64"]
  tags = RELEASE_VERSION == "" ? [format("%s:dev-orin-jp6.2.1", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"))] : [format("%s:%s-orin-jp6.2.1-l4t36.4.3", APOLLO_REPO != "" ? APOLLO_REPO : (GEOLOC == "cn" ? "registry.cn-hangzhou.aliyuncs.com/wheelos/apollo" : "wheelos/apollo"), RELEASE_VERSION)]
  cache-from = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-orin-jp621-l4t3643"]
  cache-to = CACHE_REGISTRY == "" ? [] : ["type=registry,ref=${CACHE_REGISTRY}/apollo:cache-dev-orin-jp621-l4t3643,mode=max"]
}
