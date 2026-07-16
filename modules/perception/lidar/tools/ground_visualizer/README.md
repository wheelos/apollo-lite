# Ground Segmentation Visualizer

## Overview

Web-based visualization tool to analyze and debug ground segmentation algorithm on PCD files. Uses Three.js for 3D point cloud visualization and calls the real C++ `ground_analyzer` tool via JSON API for accurate ground detection results.

## Requirements

- Python 3.6+ with Flask
- Built ground_analyzer binary
- Web browser for 3D visualization

## Build

```bash
# Build the ground_analyzer tool
./apollo.sh build_cpu perception/lidar/tools/ground_visualizer/ground_analyzer
```

## Usage

### Starting the Web Server

```bash
# Navigate to the tool directory
cd modules/perception/lidar/tools/ground_visualizer

# Start the Flask server
python3 ground_visualizer_web.py
```

The server will start on http://localhost:5000

### Using the Web Interface

1. **Open in Browser**: Navigate to http://localhost:5000
2. **Upload PCD File**: Click "Choose PCD File" and select your point cloud file
3. **Adjust Parameters**: Use sliders to tune ground segmentation parameters in real-time
4. **Run Segmentation**: Click "Run Segmentation" to process
5. **View Results**: 3D visualization shows ground (green) and non-ground (red) points
6. **Debug Info**: Check browser console for detailed detection statistics

### Command Line Usage

You can also run ground_analyzer directly:

```bash
# Basic usage
./bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer \
    /apollo/data/confs/zhongtie/debug_ground/ground_1776063769.125587_5.pcd

# With custom parameters (JSON)
./bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer \
    input.pcd --params custom_params.json

# Save results to PCD files
./bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer \
    input.pcd --save_pcd out/result

# JSON output mode
./bazel-bin/modules/perception/lidar/tools/ground_visualizer/ground_analyzer \
    input.pcd --output_json
```

## What It Shows

### 3D Visualization

- **Green Points**: Ground points detected by the algorithm
- **Red Points**: Non-ground points (obstacles)
- **Camera View**: Rear-top view of the scene (camera positioned at Y=50, Z=-100 in Apollo coords)
- **Point Cloud Centered**: Automatically centered for better visualization

### Debug Output

When running with `--output_json`, the tool provides detailed detection statistics:

1. **Input Statistics**: Point count, bounding box, coordinate system info
2. **Parameter Values**: All ground segmentation parameters used
3. **ROI Filtering**: Number of points before/after ROI filtering
4. **Grid Distribution**: Number of occupied cells, points per cell statistics
5. **Plane Fitting Results**: 
   - Cells with successful plane fitting
   - Cells with insufficient candidates
   - Support point statistics
6. **Ground/Non-Ground Counts**: Final segmentation results

## Coordinate Systems

### Apollo Coordinate System
- **X**: Right
- **Y**: Forward  
- **Z**: Up

### Three.js Coordinate System (Visualization)
- **X**: Right (same as Apollo X)
- **Y**: Up (Apollo Z)
- **Z**: Forward (Apollo Y)

**Transformation**: `positions.push(p[0], p[2], p[1])` converts Apollo (x,y,z) to Three.js (x,y,z)

## Key Parameters

### Critical for Uphill Terrain

**sample_region_z_upper** (default: 0.25m, recommend: 1.0-3.0m)
- Filters points by Z height in the near region (±32m from vehicle)
- Points above this threshold are excluded from ground detection candidates
- **Issue**: If set too low (< 0.5m), valid ground points on uphill terrain are filtered out
- **Solution**: Increase to 1.0m or higher for uphill scenarios

**sample_region_z_lower** (default: -2.0m)
- Lower Z bound for candidate point filtering
- Should be below expected ground level

### Plane Fitting Thresholds

**planefit_dist_thres_near** (default: 0.25m)
- RANSAC inlier threshold for near region
- Points within this distance from fitted plane are considered ground

**planefit_dist_thres_far** (default: 0.40m)
- RANSAC inlier threshold for far region
- Typically larger than near threshold due to point sparsity

**planefit_orien_threshold** (default: 30.0°)
- Maximum allowed plane orientation deviation from horizontal
- Used to filter unlikely ground planes

### ROI Settings

**roi_region_rad_x**, **roi_region_rad_y**, **roi_region_rad_z**
- Region of interest bounds in meters
- Points outside this region are filtered before processing

**roi_near_rad** (default: 32.0m)
- Defines the "near region" where different thresholds apply

### Grid Configuration

**nr_grids_coarse** (default: 128)
- Number of grid cells along each axis for coarse grid
- Larger values = smaller cells = more precise but slower

**nr_grids_fine** (default: 512)
- Fine grid resolution for final ground height computation

## Understanding Ground Detection

The algorithm uses a grid-based RANSAC approach:

1. **Grid Division**: Point cloud divided into 2D grid cells (X-Y plane)
2. **Candidate Selection**: Each cell selects points within Z bounds as candidates
3. **Plane Fitting**: RANSAC fits a plane to candidates in each cell
4. **Neighbor Smoothing**: Plane parameters smoothed with neighboring cells
5. **Ground Classification**: Points classified based on distance to fitted plane

### Common Issues

**Few/no ground points detected:**
- Check `sample_region_z_upper` - may be filtering valid ground points
- Check `planefit_dist_thres_near` - may be too strict
- Verify ROI settings match your point cloud extent

**Too many false ground points:**
- Decrease `planefit_dist_thres_near/far`
- Decrease `sample_region_z_upper`
- Check `planefit_orien_threshold` - may need to be lower

**Poor performance on slopes:**
- Increase `planefit_orien_threshold` to allow steeper slopes
- Increase `planefit_dist_thres` to accommodate slope height variation
- Consider using higher grid resolution for better slope modeling

## Web Interface Controls

### Mouse Controls
- **Left Click + Drag**: Rotate view
- **Mouse Wheel**: Zoom in/out
- **Right Click + Drag**: Pan view

### Parameter Adjustment
- All parameters adjustable via sliders
- Changes applied immediately on next "Run Segmentation" click
- Current parameter values shown in real-time

### Vehicle Pose
- Optional vehicle position (x, y, z) can be specified
- Used for coordinate transformation if needed
- Defaults to (0, 0, 0) if not provided

## Troubleshooting

### Server won't start
```bash
# Install Flask if needed
pip3 install flask
```

### "0 ground points" result
1. Check parameter values in browser console debug output
2. Verify `sample_region_z_upper` is not too low
3. Confirm ROI settings include your point cloud
4. Try with default parameters first

### Visualization issues
- **Black screen**: Point cloud may be outside camera view - check console for point count
- **Wrong colors**: Clear browser cache and reload
- **Slow performance**: Large PCD files (>100K points) may lag - consider downsampling

## Example Session

```
$ cd modules/perception/lidar/tools/ground_visualizer
$ python3 ground_visualizer_web.py
 * Running on http://0.0.0.0:5000

# In browser, open http://localhost:5000
# Upload PCD file -> Adjust sample_region_z_upper to 1.5
# Click "Run Segmentation"
# Check browser console for debug output:

Ground Segmentation Debug:
  Input points: 45000
  After ROI filter: 42356
  Occupied cells: 342/4096
  Successful fits: 287
  Failed fits (insufficient candidates): 55
  Ground points: 28456
  Non-ground points: 13900
```

## File Structure

```
ground_visualizer/
├── ground_analyzer.cc          # C++ analysis tool
├── ground_visualizer_web.py    # Flask web server
├── visualizer.html             # Web interface
├── BUILD                       # Bazel build config
└── README.md                   # This file
```
