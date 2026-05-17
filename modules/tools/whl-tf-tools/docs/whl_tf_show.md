# whl-tf-show

Visualize TF (transform) configuration files showing the coordinate frame relationships with interactive 3D visualization.

## Features

- Load multiple TF configuration YAML files
- Visualize coordinate frames in 3D space
- Interactive mouse controls for zooming, rotating, and panning
- Auto-detection of root frame
- Customizable frame axis length
- Save visualization to image file (PNG, PDF, SVG)

## Installation

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## Usage

**Note**: All commands assume you are running from the **project root directory**.

### Basic Usage

```bash
./modules/tools/whl-tf-tools/whl_tf_show.py modules/transform/params/*.yaml
```

### Command-Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-r, --root TEXT` | Root frame to use as reference | Auto-detect |
| `-l, --length FLOAT` | Length of coordinate axis arrows | 0.5 |
| `-s, --save PATH` | Save figure to file instead of displaying | Display |

### Examples

```bash
# Visualize specific TF files
./modules/tools/whl-tf-tools/whl_tf_show.py imu_to_localization.yaml main_front_to_imu.yaml

# Use a specific root frame
./modules/tools/whl-tf-tools/whl_tf_show.py -r localization modules/transform/params/*.yaml

# Custom axis length
./modules/tools/whl-tf-tools/whl_tf_show.py -l 1.0 modules/transform/params/*.yaml

# Save to PNG file
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.png modules/transform/params/*.yaml

# Save to high-resolution PDF
./modules/tools/whl-tf-tools/whl_tf_show.py -s output.pdf modules/transform/params/*.yaml
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

## Visualization Guide

### Color Legend

| Color | Axis |
|-------|------|
| Red | X axis |
| Green | Y axis |
| Blue | Z axis |
| Gray dashed arrow | Transform relationship between frames |

### Mouse Controls

| Action | Control |
|--------|---------|
| Rotate view | Left click + drag |
| Zoom in/out | Right click + drag |
| Pan view | Middle click + drag |
