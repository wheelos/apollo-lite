<div align="center">

<h1>
  <a href="#"><img src="docs/images/wheelos.jpg" width="100%" alt="WheelOS"/></a>
</h1>

**Apollo-Lite 低速自动驾驶产品集成仓库**

[English](README.md) | [中文](README.zh-cn.md)

[![代码检查](https://github.com/wheelos/apollo-lite/actions/workflows/lint-format.yml/badge.svg?branch=main)](https://github.com/wheelos/apollo-lite/actions/workflows/lint-format.yml)
[![许可证](https://img.shields.io/github/license/wheelos/apollo-lite)](LICENSE)

</div>

## 仓库定位

Apollo-Lite 是低速自动驾驶系统的产品集成根仓库。它通过 Bzlmod 组合独立版本化的
硬件、感知、定位、规划、控制、运行时和工具仓库。

本仓库负责：

- 低速产品配置与启动拓扑；
- 硬件、算法模块和运行时产品的选择；
- Ubuntu 开发容器与宿主机环境；
- 运行时组装、打包、安装和跨模块测试；
- 开发与 agent 协作流程。

算法模块的实现仍由其独立源码仓库负责，仓库清单见
[`wheelos.repos`](wheelos.repos)。

## 按目标进入文档

| 目标 | 入口 |
| --- | --- |
| 准备宿主机并进入开发容器 | [环境准备](docs/setup.md) |
| 预览或构建产品档位 | [产品构建](tools/products/README.md#legacy-build-inventories) |
| 查看低速产品定义 | [products/low_speed](products/low_speed/BUILD) |
| 构建完整运行时产品 | [运行时产品](tools/products/README.md#runtime-products-one-component-or-many) |
| 选择硬件或传感器后端 | [硬件构建](tools/hardware/README.md) |
| 选择算法模块 | [算法模块](tools/modules/README.md) |
| 组装统一的 Cyber 系统 | [系统组装](tools/system/README.md) |
| 使用调试和运维工具 | [工具文档](modules/tools/README.md) |
| 参与开发或定位故障 | [Agent Guide](AGENTS.md) |
| 浏览全部仓库文档 | [文档地图](docs/README.md) |

## 快速开始

环境要求：

- Ubuntu 22.04 宿主机并安装 Docker，或使用
  [`setup_host.sh`](docker/setup_host/setup_host.sh) 完成等价配置；
- 建议至少 8 核 CPU 和 8 GB 内存；
- GPU 流程建议使用 NVIDIA Turing 或更新架构；
- 使用本地源码覆盖时，需要按 [`wheelos.repos`](wheelos.repos) 准备外部仓库。

准备宿主机并进入开发容器：

```bash
sudo bash docker/setup_host/setup_host.sh
whl start dev
whl enter dev
```

先预览低速产品构建清单，确认配置和模块后再编译：

```bash
bash apollo.sh profiles
bash apollo.sh build --product=devkit-x86 --show
```

构建并运行显式的低速运行时产品：

```bash
bash apollo.sh build \
  --product=//products/low_speed:devkit_x86 \
  --config=product_devkit-x86 \
  --show

bash apollo.sh run \
  --product=//products/low_speed:devkit_x86 \
  --config=product_devkit-x86 \
  -- --launch launch/low_speed/devkit_x86.launch
```

`devkit-x86` 是构建档位；`//products/low_speed:devkit_x86` 是可执行、可打包和可安装的
运行时产品标签。二者用途不同，构建成功也不代表完成部署资格认证。

进行本地交付验证时增加 `--source=dev`：

```bash
bash apollo.sh package \
  --product=//products/low_speed:devkit_x86 \
  --source=dev \
  --config=product_devkit-x86

bash apollo.sh install \
  --product=//products/low_speed:devkit_x86 \
  --source=dev \
  --config=product_devkit-x86 \
  --destdir=/absolute/install/path
```

在车辆、传感器、标定、Cyber 域、消息通道和安全配置经过审核前，不应启动车辆或传感器程序。

## 目录职责

| 路径 | 职责 |
| --- | --- |
| [`products/low_speed/`](products/low_speed/) | 低速产品默认配置、车辆参数模板和启动拓扑 |
| [`tools/products/`](tools/products/README.md) | 面向用户的产品档位和 `runtime_product` 组装 |
| [`tools/hardware/`](tools/hardware/README.md) | 硬件配置选择、运行时组装和打包 |
| [`tools/modules/`](tools/modules/README.md) | 跨外部仓库选择算法模块 |
| [`tools/system/`](tools/system/README.md) | 使用显式模块组装统一 Core、工具和配置的系统 |
| [`tools/transform/`](tools/transform/README.md) | Transform 运行时组装和 service 示例 |
| [`docker/`](docker/) | 宿主机配置、镜像和容器生命周期 |
| [`scripts/`](scripts/) | 构建、运行时、CI 和 agent 工具 |
| [`modules/tools/`](modules/tools/README.md) | 过渡期运维和调试工具 |
| [`.agents/`](.agents/README.md) | 任务流程与工作流规范 |
| [`docs/`](docs/README.md) | 文档地图和环境准备说明 |

## 范围与安全边界

所有 `devkit-*` 产品默认都是集成示例档位，除非针对具体部署完成了显式验收，否则不构成
车辆安全认证。

- 安装过程不会启动任何组件。
- 硬件和传感器启动需要经过审核的 DAG、配置与标定。
- 正式交付必须使用不可变源码版本和正式 Bzlmod registry。
- 道路、HIL 和车辆执行器操作必须经过显式批准并遵循安全流程。

## 文档与参与开发

- [文档地图](docs/README.md)
- [环境准备与容器生命周期](docs/setup.md)
- [构建与运行时产品](tools/products/README.md)
- [硬件配置](tools/hardware/README.md)
- [Agent Guide](AGENTS.md)
- [Skills](.agents/skills/README.md)
- [工作流](.agents/workflows/README.md)
- [Agent 工具](scripts/agents/README.md)
- [问题反馈](https://github.com/wheelos/apollo-lite/issues)

## 许可证

Apollo-Lite 使用 [Apache License 2.0](LICENSE)。

合作与商务联系：`daohu527@gmail.com`。
