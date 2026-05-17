# whl-tf-tools

TF (transform) related tools for Apollo-Lite autonomous driving system.

## Installation

```bash
pip install -r modules/tools/whl-tf-tools/requirements.txt
```

## Available Tools

| Tool | Description |
|------|-------------|
| [whl-tf-show](docs/whl_tf_show.md) | Visualize TF configurations with interactive 3D visualization |
| [whl-tf-query](docs/whl_tf_query.md) | Query transforms between coordinate frames |
| [whl-tf-trans](docs/whl_tf_trans.md) | Transform coordinates between global and local vehicle frames |
| [whl-tf-to-matrix](docs/whl_tf_to_matrix.md) | Convert TF to 4x4 homogeneous transformation matrix |

## Quick Start

See [Quick Start Guide](docs/quickstart.md) for basic usage examples.

## TF Configuration Format

All tools expect YAML files with the following format:

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
