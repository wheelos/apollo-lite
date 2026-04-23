# whl-tf-trans

Coordinate transform tool - transforms coordinates between global (UTM/map) and local vehicle frames.

## Local Frame Definition

| Axis | Description |
|------|-------------|
| Origin | Current vehicle position (curr_x, curr_y) |
| Y-axis | Vehicle heading direction (forward = positive) |
| X-axis | Perpendicular to heading (left = positive) |

## Installation

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## Usage

**Note**: All commands assume you are running from the **project root directory**.

### Basic Usage (Global -> Local)

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086
```

### Inverse Transformation (Local -> Global)

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52
```

### CSV Output

```bash
# With header
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv

# Porcelain mode (no header)
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --porcelain

# Heading in degrees
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 90 \
    -x 272091.94 -y 4020872.85 --heading 177 --heading-unit deg
```

## Command-Line Options

| Option | Description | Unit | Default |
|--------|-------------|------|---------|
| `--curr-x` | Current vehicle X position | meters | - |
| `--curr-y` | Current vehicle Y position | meters | - |
| `--curr-heading` | Current vehicle heading | radians | - |
| `-x, --target-x` | Target X position | meters | - |
| `-y, --target-y` | Target Y position | meters | - |
| `--heading` | Target heading | radians | - |
| `-i, --inverse` | Inverse transformation (local -> global) | - | False |
| `--heading-unit` | Heading unit (rad/deg) | rad | rad |
| `--output-format` | Output format (text/csv) | text | text |
| `--porcelain` | Silent mode, output data only | - | False |

## Output Explanation

### Local Frame Result (Global -> Local)

| Output | Description |
|--------|-------------|
| X coordinate | Positive = left of vehicle, Negative = right |
| Y coordinate | Positive = ahead of vehicle, Negative = behind |
| Heading | Angle relative to vehicle heading |

### Global Frame Result (Local -> Global via `--inverse`)

| Output | Description |
|--------|-------------|
| X coordinate | Global UTM/map X coordinate |
| Y coordinate | Global UTM/map Y coordinate |
| Heading | Global heading angle |

## Usage Examples

### Example 1: Basic Usage

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086
```

Output:
```
============================================================
Local Frame Result (Vehicle Local Frame)
============================================================
  X coordinate (Left+ Right-):  -23.4381 m
  Y coordinate (Front+ Back-):  23.2169 m
  Heading (relative):            1.5159 rad =    86.87°
============================================================
```

### Example 2: CSV Output

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv
```

Output:
```
x,y,heading_rad
-23.4381,23.2169,1.5159
```

### Example 3: Inverse Transformation

```bash
./modules/tools/whl-tf-tools/whl_tf_trans.py --inverse \
    --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 \
    -x -23.44 -y 23.22 --heading 1.52
```

This converts local coordinates (relative to vehicle) back to global coordinates.
