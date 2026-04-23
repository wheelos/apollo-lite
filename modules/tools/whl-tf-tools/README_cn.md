# whl-tf-tools

Apollo-Lite 自动驾驶系统的 TF (transform) 相关工具集合。

## 安装

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## 可用工具

| 工具 | 描述 |
|------|------|
| [whl-tf-show](docs/whl_tf_show_cn.md) | 可视化 TF 配置，支持交互式 3D 可视化 |
| [whl-tf-query](docs/whl_tf_query_cn.md) | 查询坐标系之间的变换关系 |
| [whl-tf-trans](docs/whl_tf_trans_cn.md) | 全局坐标系与车辆局部坐标系之间的坐标转换 |
| [whl-tf-to-matrix](docs/whl_tf_to_matrix_cn.md) | 将 TF 转换为 4x4 齐次变换矩阵 |

## 快速入门

参见 [快速入门指南](docs/quickstart_cn.md) 了解基本用法。

## TF 配置文件格式

所有工具期望的 YAML 文件格式：

```yaml
header:
  frame_id: parent_frame
transform:
  translation:
    x: 0.0
    y: 1.5
    z: 0.5
  rotation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707
child_frame_id: child_frame
```
