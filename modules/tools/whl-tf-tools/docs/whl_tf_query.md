# whl-tf-query

Interactive TF query tool - load TF configuration files and query transforms between any two coordinate frames.

## Features

- Load multiple TF YAML files
- Load from Apollo static transform config files (pb.txt format)
- Query transforms between any two frames in the TF tree
- Automatic path finding through the transform graph
- Interactive query mode for multiple queries
- Save and load transform graphs for reuse
- Multiple output formats (YAML, matrix)

## Installation

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## Usage

**Note**: All commands assume you are running from the **project root directory**.

### Load Command

Load and display TF configuration files:

```bash
# Load individual TF YAML files
./modules/tools/whl-tf-tools/whl_tf_query.py load tf1.yaml tf2.yaml

# Load from Apollo static transform config
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt

# Combine both sources
./modules/tools/whl-tf-tools/whl_tf_query.py load custom.yaml -c modules/transform/conf/static_transform_conf.pb.txt

# List all loaded frames and transforms
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt --list

# Save loaded graph for later use
./modules/tools/whl-tf-tools/whl_tf_query.py load -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl
```

#### Load Options

| Option | Description |
|--------|-------------|
| `-t TF_FILES` | Individual TF YAML files to load (can specify multiple) |
| `-c, --config CONFIG_FILES` | Apollo static transform config files (pb.txt format) |
| `-a, --apollo-root PATH` | Apollo root directory (auto-detected if not specified) |
| `-o, --output PATH` | Save loaded graph to pickle file for later use |
| `--list` | List all loaded frames and transforms |

### Query Command

Query transform between two frames:

```bash
# Query transform with direct files
./modules/tools/whl-tf-tools/whl_tf_query.py query -t tf1.yaml -t tf2.yaml imu rslidar_main_front

# Query transform from config
./modules/tools/whl-tf-tools/whl_tf_query.py query -c modules/transform/conf/static_transform_conf.pb.txt rslidar_main_front rslidar_main_rear

# Query with saved graph
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization

# Output transform to YAML file
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization -o imu_to_localization.yaml

# Output as transformation matrix
./modules/tools/whl-tf-tools/whl_tf_query.py query -t tf1.yaml -t tf2.yaml imu rslidar -f matrix

# Output all formats
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu localization -f all
```

#### Query Options

| Option | Description | Default |
|--------|-------------|---------|
| `-t TF_FILES` | TF YAML files (same as load command) | - |
| `-c, --config CONFIG_FILES` | Config files (same as load command) | - |
| `-a, --apollo-root PATH` | Apollo root directory | Auto-detect |
| `-g, --graph-file PATH` | Load previously saved graph | - |
| `-o, --output PATH` | Output transform to YAML file | - |
| `-f, --format FORMAT` | Output format (yaml, matrix, all) | yaml |
| `-p, --precision INT` | Decimal precision for matrix output | 6 |

### Interactive Command

Interactive mode for querying multiple transforms:

```bash
# Start interactive mode with files
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -t tf1.yaml -t tf2.yaml

# Start with saved graph
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -g graph.pkl

# Start with Apollo config
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -c modules/transform/conf/static_transform_conf.pb.txt
```

#### Interactive Commands

Once in interactive mode:

| Command | Description |
|---------|-------------|
| `query FROM TO` | Query transform FROM <- TO |
| `list` | List all frames and transforms |
| `help` | Show help message |
| `quit` or `exit` or `q` | Exit interactive mode |

#### Example Interactive Session

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

## TF Configuration File Format

The tool expects YAML files with the following format:

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

## Usage Examples

### Example 1: Query Lidar to IMU Transform

```bash
./modules/tools/whl-tf-tools/whl_tf_query.py query \
    -c modules/transform/conf/static_transform_conf.pb.txt \
    rslidar_main_front imu
```

Output:
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

### Example 2: Save and Reuse Transform Graph

```bash
# First, load and save the graph
./modules/tools/whl-tf-tools/whl_tf_query.py load \
    -c modules/transform/conf/static_transform_conf.pb.txt -o graph.pkl

# Later, use the saved graph for queries
./modules/tools/whl-tf-tools/whl_tf_query.py query -g graph.pkl imu rslidar_main_rear
```

### Example 3: Batch Processing with Interactive Mode

```bash
./modules/tools/whl-tf-tools/whl_tf_query.py interactive -g graph.pkl
> query imu localization
> query imu rslidar_main_front
> query imu rslidar_main_rear
> quit
```
