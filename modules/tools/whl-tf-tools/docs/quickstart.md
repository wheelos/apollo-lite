# whl-tf-tools Quick Start

This guide covers common use cases for all tools.

**Note**: All commands assume you are in the **project root directory**.

## whl-tf-show - Visualize TF Configurations

```bash
# Visualize all TF configs in a directory
./modules/tools/whl-tf-tools/whl_tf_show.py modules/transform/params/*.yaml

# Save to PNG
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.png modules/transform/params/*.yaml

# Custom axis length
./modules/tools/whl-tf-tools/whl_tf_show.py -l 1.0 modules/transform/params/*.yaml
```

## whl-tf-query - Query Transforms

```bash
# Query transform from Apollo config
./modules/tools/whl-tf-tools/whl_tf_query.py query \
    -c modules/transform/conf/static_transform_conf.pb.txt \
    imu rslidar_main_front

# Interactive mode
./modules/tools/whl-tf-tools/whl_tf_query.py interactive \
    -c modules/transform/conf/static_transform_conf.pb.txt

# Save and reuse graph
./modules/tools/whl-tf-tools/whl_tf_query.py load \
    -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization
```

## whl-tf-to-matrix - Convert to Matrix

```bash
# From YAML file
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py file /path/to/tf.yaml

# From command line (quaternion)
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1.0 --ty 2.0 --tz 3.0 --qx 0 --qy 0 --qz 0 --qw 1

# From command line (Euler angles, degrees)
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 90 --pitch 0 --yaw 0 --degrees

# Output as Python code
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f python
```

## whl-tf-trans - Coordinate Transform

```bash
# Global to local vehicle frame
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086

# Local to global (inverse)
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52

# CSV output
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv
```

## See Also

- [whl-tf-show Detailed Guide](whl_tf_show.md)
- [whl-tf-query Detailed Guide](whl_tf_query.md)
- [whl-tf-to-matrix Detailed Guide](whl_tf_to_matrix.md)
- [whl-tf-trans Detailed Guide](whl_tf_trans.md)
