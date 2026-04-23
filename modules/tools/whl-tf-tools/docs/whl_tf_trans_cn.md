# whl-tf-trans

坐标转换工具 - 在全局坐标系（UTM/map）和车辆局部坐标系之间进行坐标转换。

## 局部坐标系定义

| 坐标轴 | 说明 |
|--------|------|
| 原点 | 当前车辆位置 (curr_x, curr_y) |
| Y轴 | 车辆航向方向（前方为正） |
| X轴 | 垂直于航向（左侧为正） |

## 安装

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## 使用方法

**注意**：所有命令都假设您在**项目根目录**下运行。

### 基本用法（全局 -> 局部）

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086
```

### 反向变换（局部 -> 全局）

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52
```

### CSV 输出

```bash
# 带表头
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv

# 静默模式（无表头）
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --porcelain

# 航向使用角度制
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 90 \
    -x 272091.94 -y 4020872.85 --heading 177 --heading-unit deg
```

## 命令行选项

| 选项 | 说明 | 单位 | 默认值 |
|------|------|------|--------|
| `--curr-x` | 当前车辆 X 坐标 | 米 | - |
| `--curr-y` | 当前车辆 Y 坐标 | 米 | - |
| `--curr-heading` | 当前车辆航向 | 弧度 | - |
| `-x, --target-x` | 目标点 X 坐标 | 米 | - |
| `-y, --target-y` | 目标点 Y 坐标 | 米 | - |
| `--heading` | 目标点航向 | 弧度 | - |
| `-i, --inverse` | 反向变换：局部 -> 全局 | - | False |
| `--heading-unit` | 航向单位 (rad/deg) | rad | rad |
| `--output-format` | 输出格式 (text/csv) | text | text |
| `--porcelain` | 静默模式，仅输出数据 | - | False |

## 输出说明

### 局部坐标系结果（全局 -> 局部）

| 输出 | 说明 |
|------|------|
| X坐标 | 正值表示在车辆左侧，负值表示在右侧 |
| Y坐标 | 正值表示在车辆前方，负值表示在后方 |
| 航向角 | 相对于车辆航向的角度差 |

### 全局坐标系结果（局部 -> 全局，通过 `--inverse`）

| 输出 | 说明 |
|------|------|
| X坐标 | 全局 UTM/map X 坐标 |
| Y坐标 | 全局 UTM/map Y 坐标 |
| 航向角 | 全局航向角度 |

## 使用示例

### 示例 1: 基本用法

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086
```

输出：
```
============================================================
Local Frame Result (Vehicle Local Frame)
============================================================
  X coordinate (Left+ Right-):  -23.4381 m
  Y coordinate (Front+ Back-):  23.2169 m
  Heading (relative):            1.5159 rad =    86.87°
============================================================
```

### 示例 2: CSV 输出

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv
```

输出：
```
x,y,heading_rad
-23.4381,23.2169,1.5159
```

### 示例 3: 反向变换

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52
```

这将局部坐标（相对于车辆）转换回全局坐标。
