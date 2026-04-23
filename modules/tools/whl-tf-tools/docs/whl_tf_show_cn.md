# whl-tf-show

可视化 TF (transform) 配置文件，以交互式 3D 可视化展示坐标系关系。

## 功能特性

- 加载多个 TF 配置 YAML 文件
- 在 3D 空间中可视化坐标系
- 交互式鼠标控制缩放、旋转和平移
- 自动检测根坐标系
- 可自定义坐标轴长度
- 保存可视化结果到图片文件（PNG、PDF、SVG）

## 安装

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## 使用方法

**注意**：所有命令都假设您在**项目根目录**下运行。

### 基本用法

```bash
./modules/tools/whl-tf-tools/whl_tf_show.py modules/transform/params/*.yaml
```

### 命令行选项

| 选项 | 说明 | 默认值 |
|------|------|--------|
| `-r, --root TEXT` | 作为参考的根坐标系 | 自动检测 |
| `-l, --length FLOAT` | 坐标轴箭头长度 | 0.5 |
| `-s, --save PATH` | 保存图形到文件而不是显示 | 显示 |

### 示例

```bash
# 可视化指定的 TF 文件
./modules/tools/whl-tf-tools/whl_tf_show.py imu_to_localization.yaml main_front_to_imu.yaml

# 使用指定的根坐标系
./modules/tools/whl-tf-tools/whl_tf_show.py -r localization modules/transform/params/*.yaml

# 自定义坐标轴长度
./modules/tools/whl-tf-tools/whl_tf_show.py -l 1.0 modules/transform/params/*.yaml

# 保存为 PNG 文件
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.png modules/transform/params/*.yaml

# 保存为高分辨率 PDF
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.pdf modules/transform/params/*.yaml
```

## TF 配置文件格式

工具期望的 YAML 文件格式：

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

## 可视化说明

### 颜色图例

| 颜色 | 坐标轴 |
|------|--------|
| 红色 | X 轴 |
| 绿色 | Y 轴 |
| 蓝色 | Z 轴 |
| 灰色虚线箭头 | 坐标系之间的变换关系 |

### 鼠标控制

| 操作 | 控制 |
|------|------|
| 旋转视图 | 左键拖拽 |
| 缩放 | 右键拖拽 |
| 平移视图 | 中键拖拽 |
