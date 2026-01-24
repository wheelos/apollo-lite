# whl-tf-tools 快速入门

本指南涵盖所有工具的常用场景。

**注意**：所有命令都假设您在**项目根目录**下运行。

## whl-tf-show - 可视化 TF 配置

```bash
# 可视化目录中的所有 TF 配置
./modules/tools/whl-tf-tools/whl_tf_show.py modules/transform/params/*.yaml

# 保存为 PNG
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.png modules/transform/params/*.yaml

# 自定义坐标轴长度
./modules/tools/whl-tf-tools/whl_tf_show.py -l 1.0 modules/transform/params/*.yaml
```

## whl-tf-query - 查询变换

```bash
# 从 Apollo 配置查询变换
./modules/tools/whl-tf-tools/whl_tf_query.py query \
    -c modules/transform/conf/static_transform_conf.pb.txt \
    imu rslidar_main_front

# 交互式模式
./modules/tools/whl-tf-tools/whl_tf_query.py interactive \
    -c modules/transform/conf/static_transform_conf.pb.txt

# 保存并复用图
./modules/tools/whl-tf-tools/whl_tf_query.py load \
    -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization
```

## whl-tf-to-matrix - 转换为矩阵

```bash
# 从 YAML 文件
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py file /path/to/tf.yaml

# 从命令行（四元数）
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1.0 --ty 2.0 --tz 3.0 --qx 0 --qy 0 --qz 0 --qw 1

# 从命令行（欧拉角，角度制）
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 90 --pitch 0 --yaw 0 --degrees

# 输出为 Python 代码
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f python
```

## whl-tf-trans - 坐标转换

```bash
# 全局坐标系 -> 车辆局部坐标系
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086

# 局部坐标系 -> 全局坐标系（反向）
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52

# CSV 输出
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv
```

## 相关文档

- [whl-tf-show 详细指南](whl_tf_show_cn.md)
- [whl-tf-query 详细指南](whl_tf_query_cn.md)
- [whl-tf-to-matrix 详细指南](whl_tf_to_matrix_cn.md)
- [whl-tf-trans 详细指南](whl_tf_trans_cn.md)
