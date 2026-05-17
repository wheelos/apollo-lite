# whl-tf-to-matrix

将 TF (transform) 配置转换为 4x4 齐次变换矩阵。

## 功能特性

- 从 YAML TF 文件转换
- 从命令行参数转换（四元数或欧拉角）
- 多种输出格式：表格、Python 代码、C++ Eigen 代码
- 精确的小数控制

## 安装

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## 使用方法

**注意**：所有命令都假设您在**项目根目录**下运行。

### 从 YAML 文件

```bash
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py file /path/to/tf.yaml
```

### 从命令行（四元数）

```bash
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1.0 --ty 2.0 --tz 3.0 --qx 0 --qy 0 --qz 0 --qw 1
```

#### 四元数参数

| 参数 | 说明 |
|------|------|
| `--tx, --ty, --tz` | 平移量 (x, y, z)，单位：米 |
| `--qx, --qy, --qz, --qw` | 四元数旋转 (x, y, z, w) |

### 从命令行（欧拉角）

```bash
# 弧度（默认）
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 0.1 --pitch 0.2 --yaw 0.3

# 角度
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 90 --pitch 0 --yaw 0 --degrees
```

#### 欧拉角参数

| 参数 | 说明 |
|------|------|
| `--tx, --ty, --tz` | 平移量 (x, y, z)，单位：米 |
| `--roll, --pitch, --yaw` | 旋转角（横滚-俯仰-偏航顺序） |
| `--degrees` | 欧拉角使用角度而非弧度 |

### 输出格式

```bash
# 表格格式（默认）
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1

# Python 代码
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f python

# C++ Eigen 代码
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f cpp
```

### 命令行选项

| 选项 | 说明 | 默认值 |
|------|------|--------|
| `-f, --format` | 输出格式 (table, python, cpp) | table |
| `-p, --precision` | 小数精度 | 6 |
| `--degrees` | 欧拉角使用角度而非弧度 | False (弧度) |

## 输出示例

### 表格格式

```
        |          X |         Y |        Z |
--------|-----------|----------|---------|
X       |   1.000000 |  0.000000 | 0.000000 |
Y       |   0.000000 |  1.000000 | 0.000000 |
Z       |   0.000000 |  0.000000 | 1.000000 |
Translation | 1.000000 |  0.000000 | 3.000000 |
```

### Python 格式

```python
import numpy as np

matrix = np.array([
    [1.000000, 0.000000, 0.000000, 1.000000],
    [0.000000, 1.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 1.000000, 3.000000],
    [0.000000, 0.000000, 0.000000, 1.000000]
])
```

### C++ Eigen 格式

```cpp
#include <Eigen/Dense>

Eigen::Matrix4d matrix;
matrix << 1.000000, 0.000000, 0.000000, 1.000000,
           0.000000, 1.000000, 0.000000, 0.000000,
           0.000000, 0.000000, 1.000000, 3.000000,
           0.000000, 0.000000, 0.000000, 1.000000;
```
