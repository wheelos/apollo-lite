# whl-tf-query

交互式 TF 查询工具 - 加载 TF 配置文件并查询任意两个坐标系之间的变换关系。

## 功能特性

- 加载多个 TF YAML 文件
- 从 Apollo 静态变换配置文件加载（pb.txt 格式）
- 查询 TF 树中任意两个坐标系之间的变换
- 自动通过变换图查找路径
- 交互式查询模式支持多次查询
- 保存和加载变换图以供复用
- 多种输出格式（YAML、矩阵）

## 安装

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## 使用方法

**注意**：所有命令都假设您在**项目根目录**下运行。

### Load 命令

加载并显示 TF 配置文件：

```bash
# 加载单个 TF YAML 文件
./modules/tools/whl-tf-tools/whl_tf_query.py load tf1.yaml tf2.yaml

# 从 Apollo 静态变换配置加载
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt

# 同时加载两种来源
./modules/tools/whl-tf-tools/whl_tf_query.py load custom.yaml -c modules/transform/conf/static_transform_conf.pb.txt

# 列出所有加载的坐标系和变换
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt --list

# 保存加载的图供后续使用
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl
```

#### Load 选项

| 选项 | 说明 |
|------|------|
| `-t TF_FILES` | 要加载的 TF YAML 文件（可指定多个） |
| `-c, --config CONFIG_FILES` | Apollo 静态变换配置文件（pb.txt 格式） |
| `-a, --apollo-root PATH` | Apollo 根目录（不指定则自动检测） |
| `-o, --output PATH` | 将加载的图保存到 pickle 文件供后续使用 |
| `--list` | 列出所有加载的坐标系和变换 |

### Query 命令

查询两个坐标系之间的变换：

```bash
# 使用直接文件查询变换
./modules/tools/whl-tf-tools/whl_tf_query.py query -t tf1.yaml -t tf2.yaml imu rslidar_main_front

# 从配置文件查询变换
./modules/tools/whl-tf-tools/whl_tf_query.py query -c modules/transform/conf/static_transform_conf.pb.txt rslidar_main_front rslidar_main_rear

# 使用保存的图查询
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization

# 将变换输出到 YAML 文件
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization -o imu_to_localization.yaml

# 输出为变换矩阵
./modules/tools/whl-tf-tools/whl_tf_query.py query -t tf1.yaml -t tf2.yaml imu rslidar -f matrix

# 输出所有格式
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization -f all
```

#### Query 选项

| 选项 | 说明 | 默认值 |
|------|------|--------|
| `-t TF_FILES` | TF YAML 文件（与 load 命令相同） | - |
| `-c, --config CONFIG_FILES` | 配置文件（与 load 命令相同） | - |
| `-a, --apollo-root PATH` | Apollo 根目录 | 自动检测 |
| `-g, --graph-file PATH` | 加载之前保存的图 | - |
| `-o, --output PATH` | 将变换输出到 YAML 文件 | - |
| `-f, --format FORMAT` | 输出格式（yaml、matrix、all） | yaml |
| `-p, --precision INT` | 矩阵输出的小数精度 | 6 |

### Interactive 命令

交互式模式，支持多次查询变换：

```bash
# 使用文件启动交互模式
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -t tf1.yaml -t tf2.yaml

# 使用保存的图启动
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -g graph.pkl

# 使用 Apollo 配置启动
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -c modules/transform/conf/static_transform_conf.pb.txt
```

#### 交互式命令

进入交互模式后：

| 命令 | 说明 |
|------|------|
| `query FROM TO` | 查询 FROM <- TO 的变换 |
| `list` | 列出所有坐标系和变换 |
| `help` | 显示帮助信息 |
| `quit` 或 `exit` 或 `q` | 退出交互模式 |

#### 交互式会话示例

```
> query imu localization

Transform: localization -> imu
Path: imu -> localization
Translation: (0.00000, 0.00000, 1.50000)
Quaternion (xyzw): (0.00000000, 0.00000000, 0.00000000, 1.00000000)

> list

Available frames:
  imu
  localization
  rslidar_main_front
  rslidar_main_rear

Available transforms:
  imu -> localization
  localization -> rslidar_main_front
  localization -> rslidar_main_rear

> quit
Goodbye!
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

## 使用示例

### 示例 1: 查询激光雷达到 IMU 的变换

```bash
./modules/tools/whl-tf-tools/whl_tf_query.py query \
    -c modules/transform/conf/static_transform_conf.pb.txt \
    rslidar_main_front imu
```

输出：
```
Transform: imu -> rslidar_main_front
Path: rslidar_main_front -> localization -> imu

============================================================
# proj: +proj=utm +zone=51 +ellps=WGS84
# scale:1.11177
header:
  stamp:
    secs: 1422601952
    nsecs: 288805456
  seq: 0
  frame_id: rslidar_main_front
transform:
  translation:
    x: 1.23456
    y: 0.00000
    z: 0.50000
  rotation:
    x: 0.00000000
    y: 0.00000000
    z: 0.70710678
    w: 0.70710678
child_frame_id: imu
============================================================
```

### 示例 2: 保存并复用变换图

```bash
# 首先加载并保存图
./modules/tools/whl-tf-tools/whl_tf_query.py load \
    -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl

# 之后使用保存的图进行查询
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu rslidar_main_rear
```

### 示例 3: 使用交互模式批量处理

```bash
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -g graph.pkl
> query imu localization
> query imu rslidar_main_front
> query imu rslidar_main_rear
> quit
```
