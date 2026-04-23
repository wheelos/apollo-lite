# whl-tf-to-matrix

Convert TF (transform) configuration to 4x4 homogeneous transformation matrix.

## Features

- Convert from YAML TF files
- Convert from command line arguments (quaternion or Euler angles)
- Multiple output formats: table, Python code, C++ Eigen code
- Precise decimal control

## Installation

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## Usage

**Note**: All commands assume you are running from the **project root directory**.

### From YAML File

```bash
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py file /path/to/tf.yaml
```

### From Command Line (Quaternion)

```bash
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1.0 --ty 2.0 --tz 3.0 --qx 0 --qy 0 --qz 0 --qw 1
```

#### Quaternion Parameters

| Parameter | Description |
|-----------|-------------|
| `--tx, --ty, --tz` | Translation (x, y, z) in meters |
| `--qx, --qy, --qz, --qw` | Quaternion rotation (x, y, z, w) |

### From Command Line (Euler Angles)

```bash
# Radians (default)
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 0.1 --pitch 0.2 --yaw 0.3

# Degrees
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py euler \
    --tx 0 --ty 0 --tz 0 --roll 90 --pitch 0 --yaw 0 --degrees
```

#### Euler Angle Parameters

| Parameter | Description |
|-----------|-------------|
| `--tx, --ty, --tz` | Translation (x, y, z) in meters |
| `--roll, --pitch, --yaw` | Rotation in roll-pitch-yaw order |
| `--degrees` | Use degrees instead of radians (for Euler angles only) |

### Output Formats

```bash
# Table format (default)
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1

# Python code
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f python

# C++ Eigen code
./modules/tools/whl-tf-tools/whl_tf_to_matrix.py cli \
    --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1 -f cpp
```

### Command-Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-f, --format` | Output format (table, python, cpp) | table |
| `-p, --precision` | Decimal precision | 6 |
| `--degrees` | Use degrees for Euler angles instead of radians | False (radians) |

## Output Examples

### Table Format

```
        |          X |         Y |        Z |
--------|-----------|----------|---------|
X       |   1.000000 |  0.000000 | 0.000000 |
Y       |   0.000000 |  1.000000 | 0.000000 |
Z       |   0.000000 |  0.000000 | 1.000000 |
Translation | 1.000000 |  0.000000 | 3.000000 |
```

### Python Format

```python
import numpy as np

matrix = np.array([
    [1.000000, 0.000000, 0.000000, 1.000000],
    [0.000000, 1.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 1.000000, 3.000000],
    [0.000000, 0.000000, 0.000000, 1.000000]
])
```

### C++ Eigen Format

```cpp
#include <Eigen/Dense>

Eigen::Matrix4d matrix;
matrix << 1.000000, 0.000000, 0.000000, 1.000000,
           0.000000, 1.000000, 0.000000, 0.000000,
           0.000000, 0.000000, 1.000000, 3.000000,
           0.000000, 0.000000, 0.000000, 1.000000;
```
