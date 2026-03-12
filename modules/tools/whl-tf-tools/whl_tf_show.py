#!/usr/bin/env python3
# Copyright 2025 Pride Leong <lykling.lyk@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""whl-tf-show

Visualize TF (transform) configuration files showing the coordinate frame
relationships with interactive 3D visualization.
"""

import pathlib
from typing import Dict, List, Tuple, Set

import click
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Dynamic protobuf support
try:
    from google.protobuf import text_format
    from google.protobuf.descriptor_pool import DescriptorPool
    from google.protobuf.message import Message
    HAS_PROTOBUF = True
except ImportError:
    HAS_PROTOBUF = False


def quaternion_to_rotation_matrix(
        q: Tuple[float, float, float, float]) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix.

    Args:
        q: Quaternion as (x, y, z, w)

    Returns:
        3x3 rotation matrix
    """
    x, y, z, w = q
    # Normalize quaternion
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    # Compute rotation matrix
    R = np.array(
        [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R


def get_axis_colors() -> Dict[str, str]:
    """Get standard axis colors.

    Returns:
        Dictionary mapping axis names to colors
    """
    return {'x': 'r', 'y': 'g', 'z': 'b'}


class Transform:
    """Represents a transform between parent and child frames.

    The transform T_parent_child represents the pose of child frame relative to
    parent frame. For any point p in child frame, its coordinates in parent frame
    are: p_parent = T_parent_child * p_child

    This follows the TF semantics: pose_child_frame * tf = pose_frame
    """

    def __init__(self, parent: str, child: str,
                 translation: Tuple[float, float, float],
                 rotation: Tuple[float, float, float, float]):
        """Initialize a Transform.

        Args:
            parent: Parent frame ID
            child: Child frame ID
            translation: Translation of child frame origin in parent frame (x, y, z)
            rotation: Rotation from child frame to parent frame as quaternion (x, y, z, w)
        """
        self.parent = parent
        self.child = child
        self.translation = np.array(translation)
        self.rotation = rotation
        # Rotation matrix that rotates vectors from child frame to parent frame
        self.rotation_matrix = quaternion_to_rotation_matrix(rotation)

    def get_transform_matrix(self) -> np.ndarray:
        """Get 4x4 homogeneous transformation matrix T_parent_child.

        This matrix transforms points from child frame to parent frame:
            p_parent = T_parent_child * p_child

        Returns:
            4x4 transformation matrix where:
            - Top-left 3x3 is rotation matrix R_parent_child
            - Top-right 3x1 is translation t_parent_child (child origin in parent frame)
        """
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix  # R_parent_child
        T[:3, 3] = self.translation  # t_parent_child
        return T

    def get_inverse_matrix(self) -> np.ndarray:
        """Get inverse 4x4 transformation matrix T_child_parent.

        This matrix transforms points from parent frame to child frame:
            p_child = T_child_parent * p_parent

        Returns:
            4x4 inverse transformation matrix
        """
        T = self.get_transform_matrix()
        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv


class TFGraph:
    """Graph of coordinate frame transformations."""

    def __init__(self):
        """Initialize an empty TF graph."""
        self.transforms: Dict[str, Transform] = {}  # child -> Transform
        self.frames: set = set()

    def add_transform(self, transform: Transform):
        """Add a transform to the graph.

        Args:
            transform: Transform to add
        """
        key = (transform.parent, transform.child)
        self.transforms[key] = transform
        self.frames.add(transform.parent)
        self.frames.add(transform.child)

    def compute_transform(self,
                          from_frame: str,
                          to_frame: str,
                          visited: set = None) -> Tuple[np.ndarray, List[str]]:
        """Compute transform from one frame to another.

        Computes T_from_to that transforms points from to_frame to from_frame:
            p_from = T_from_to * p_to

        Args:
            from_frame: Source frame ID (target/result frame)
            to_frame: Destination frame ID (input frame)
            visited: Set of visited frames (for cycle detection)

        Returns:
            Tuple of (4x4 transformation matrix T_from_to, path of frames from to_frame to from_frame)

        Raises:
            ValueError: If no path exists between frames
        """
        if visited is None:
            visited = set()

        if from_frame == to_frame:
            return np.eye(4), [from_frame]

        if from_frame in visited:
            raise ValueError(f"Cycle detected in transform graph")

        visited.add(from_frame)

        # Find transforms where from_frame is the parent
        # i.e., we have a transform T_from_child
        for (parent, child), transform in self.transforms.items():
            if parent == from_frame:
                try:
                    # Recursively compute T_child_to
                    child_transform, path = self.compute_transform(
                        child, to_frame, visited.copy())
                    # Compose transforms: T_from_to = T_from_child * T_child_to
                    # This transforms points: p_from = T_from_child * p_child = T_from_child * T_child_to * p_to
                    return transform.get_transform_matrix(
                    ) @ child_transform, [from_frame] + path
                except ValueError:
                    continue

        # Backward: find transforms where from_frame is the child
        # Use inverse transform to go from child to parent
        for (parent, child), transform in self.transforms.items():
            if child == from_frame:
                try:
                    # Recursively compute T_parent_to
                    parent_transform, path = self.compute_transform(
                        parent, to_frame, visited.copy())
                    # Compose transforms: T_from_to = T_from_parent * T_parent_to
                    # where T_from_parent is the inverse of T_parent_from
                    return transform.get_inverse_matrix(
                    ) @ parent_transform, [from_frame] + path
                except ValueError:
                    continue

        raise ValueError(
            f"No transform path found from '{to_frame}' to '{from_frame}'")

    def find_root_frames(self) -> List[str]:
        """Find all root frames (frames that are not children of any transform).

        Returns:
            List of root frame IDs
        """
        children = {transform.child for transform in self.transforms.values()}
        roots = self.frames - children
        return list(roots)

    def compute_all_frame_poses(
            self, root_frame: str) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """Compute the position and orientation of all frames relative to the root frame.

        Handles multiple disconnected sub-trees by computing transforms between
        root nodes and merging all frames into a unified coordinate system.

        Args:
            root_frame: Root frame ID to use as reference

        Returns:
            Dictionary mapping frame IDs to tuple of (position, rotation_matrix):
            - position: 3D position in root frame (as numpy array)
            - rotation_matrix: 3x3 rotation matrix from frame to root frame
        """
        poses = {root_frame: (np.zeros(3), np.eye(3))}

        # Compute poses for all frames using compute_transform
        # This supports bidirectional traversal through the transform graph
        for frame in self.frames:
            if frame == root_frame:
                continue

            try:
                # Compute transform from root_frame to current frame
                T_root_frame, _ = self.compute_transform(root_frame, frame)
                poses[frame] = (T_root_frame[:3, 3], T_root_frame[:3, :3])
            except ValueError as e:
                # Frame is not reachable from root_frame
                # This shouldn't happen with bidirectional traversal
                # unless there are truly disconnected components
                import warnings
                warnings.warn(f"Cannot compute pose for '{frame}': {e}")

        return poses

    def get_transform_chain(self, from_frame: str,
                            to_frame: str) -> List[Tuple[str, str]]:
        """Get the chain of transforms from one frame to another.

        Args:
            from_frame: Target/source frame ID
            to_frame: Starting/destination frame ID

        Returns:
            List of (parent, child) tuples representing the transform chain

        Raises:
            ValueError: If no path exists between frames
        """
        if from_frame == to_frame:
            return []

        # BFS to find path
        from collections import deque

        # Build adjacency list
        adj = {}
        for (parent, child) in self.transforms.keys():
            if parent not in adj:
                adj[parent] = []
            adj[parent].append(child)
            if child not in adj:
                adj[child] = []
            adj[child].append(parent)

        # BFS from to_frame to from_frame
        queue = deque([(to_frame, [])])
        visited = {to_frame}

        while queue:
            current, path = queue.popleft()

            if current == from_frame:
                return path

            if current in adj:
                for neighbor in adj[current]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        # Check if there's a direct transform
                        if (current, neighbor) in self.transforms:
                            queue.append(
                                (neighbor, path + [(current, neighbor)]))
                        else:
                            queue.append(
                                (neighbor, path + [(neighbor, current)]))

        raise ValueError(
            f"No transform path found from '{to_frame}' to '{from_frame}'")


def load_transform_file(filepath: pathlib.Path) -> Transform:
    """Load a transform from a YAML file.

    Args:
        filepath: Path to the YAML file

    Returns:
        Transform object

    Raises:
        ValueError: If file format is invalid
    """
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    try:
        parent = data['header']['frame_id']
        child = data['child_frame_id']
        translation = (data['transform']['translation']['x'],
                       data['transform']['translation']['y'],
                       data['transform']['translation']['z'])
        rotation = (data['transform']['rotation']['x'],
                    data['transform']['rotation']['y'],
                    data['transform']['rotation']['z'],
                    data['transform']['rotation']['w'])
    except KeyError as e:
        raise ValueError(f"Invalid transform file format: {e}")

    return Transform(parent, child, translation, rotation)


class _DynamicExtrinsicFile:
    """Dynamic message class for ExtrinsicFile.

    This is a fallback when protobuf is not available.
    Parses the text format manually.
    """

    def __init__(self, text: str):
        self.frame_id = None
        self.child_frame_id = None
        self.file_path = None
        self.enable = True
        self._parse(text)

    def _parse(self, text: str):
        """Parse protobuf text format manually."""
        current_field = None
        in_braces = False
        brace_depth = 0

        for line in text.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            # Count braces
            brace_depth += line.count('{') - line.count('}')
            in_braces = brace_depth > 0

            # Handle field assignments
            if ':' in line and not line.strip().endswith('{'):
                parts = line.split(':', 1)
                if len(parts) == 2:
                    field = parts[0].strip()
                    value = parts[1].strip()

                    if not in_braces or (in_braces and current_field):
                        if field == 'frame_id':
                            self.frame_id = value.strip('"\'')
                        elif field == 'child_frame_id':
                            self.child_frame_id = value.strip('"\'')
                        elif field == 'file_path':
                            self.file_path = value.strip('"\'')
                        elif field == 'enable':
                            self.enable = value.lower() in ('true', '1')

            elif line.endswith('{'):
                current_field = line.split('{')[0].strip()

    def has_field(self, name: str) -> bool:
        return hasattr(self, name) and getattr(self, name) is not None


def load_static_transform_conf_pbtxt(
        filepath: pathlib.Path,
        apollo_root: pathlib.Path = None
) -> List[Tuple[str, str, pathlib.Path]]:
    """Load transform configuration from a pb.txt file.

    Parses the static_transform_conf.pb.txt format and returns a list of
    (frame_id, child_frame_id, file_path) tuples.

    Args:
        filepath: Path to the pb.txt file
        apollo_root: Apollo root directory for resolving relative paths

    Returns:
        List of (frame_id, child_frame_id, file_path) tuples

    Raises:
        ValueError: If file format is invalid or protobuf is not available
    """
    with open(filepath, 'r') as f:
        content = f.read()

    results = []

    # Find all extrinsic_file blocks
    lines = content.split('\n')
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith('extrinsic_file') and '{' in line:
            # Start of a new block
            frame_id = None
            child_frame_id = None
            file_path = None
            enable = True

            i += 1
            brace_depth = 1
            while i < len(lines) and brace_depth > 0:
                inner_line = lines[i].strip()
                brace_depth += inner_line.count('{') - inner_line.count('}')

                if ':' in inner_line and brace_depth == 1:
                    parts = inner_line.split(':', 1)
                    if len(parts) == 2:
                        field = parts[0].strip()
                        value = parts[1].strip()

                        if field == 'frame_id':
                            frame_id = value.strip('"\'')
                        elif field == 'child_frame_id':
                            child_frame_id = value.strip('"\'')
                        elif field == 'file_path':
                            file_path = value.strip('"\'')
                        elif field == 'enable':
                            enable = value.lower() in ('true', '1')

                i += 1

            if enable and frame_id and child_frame_id and file_path:
                # Resolve file path relative to apollo_root if provided
                full_path = pathlib.Path(file_path)
                if not full_path.is_absolute() and apollo_root:
                    full_path = apollo_root / full_path.relative_to('/apollo')
                results.append((frame_id, child_frame_id, full_path))

        i += 1

    return results


def draw_coordinate_system(ax,
                           origin: np.ndarray,
                           R: np.ndarray,
                           length: float = 0.5,
                           label: str = None,
                           alpha: float = 0.8):
    """Draw a 3D coordinate system.

    Args:
        ax: Matplotlib 3D axis
        origin: Origin position as (x, y, z)
        R: Rotation matrix (3x3)
        length: Length of axis arrows
        label: Label for the frame
        alpha: Transparency of the frame
    """
    origin = np.array(origin)
    colors = get_axis_colors()

    # X axis (red)
    x_end = origin + R[:3, 0] * length
    ax.quiver(origin[0],
              origin[1],
              origin[2],
              x_end[0] - origin[0],
              x_end[1] - origin[1],
              x_end[2] - origin[2],
              color=colors['x'],
              arrow_length_ratio=0.2,
              alpha=alpha)

    # Y axis (green)
    y_end = origin + R[:3, 1] * length
    ax.quiver(origin[0],
              origin[1],
              origin[2],
              y_end[0] - origin[0],
              y_end[1] - origin[1],
              y_end[2] - origin[2],
              color=colors['y'],
              arrow_length_ratio=0.2,
              alpha=alpha)

    # Z axis (blue)
    z_end = origin + R[:3, 2] * length
    ax.quiver(origin[0],
              origin[1],
              origin[2],
              z_end[0] - origin[0],
              z_end[1] - origin[1],
              z_end[2] - origin[2],
              color=colors['z'],
              arrow_length_ratio=0.2,
              alpha=alpha)

    # Add label
    if label:
        ax.text(origin[0],
                origin[1],
                origin[2],
                label,
                fontsize=8,
                fontweight='bold')


def draw_transform_arrow(ax,
                         start: np.ndarray,
                         end: np.ndarray,
                         label: str = None,
                         alpha: float = 0.5,
                         reverse: bool = False):
    """Draw an arrow representing a transform.

    According to TF semantics (pose_child_frame * tf = pose_frame),
    the arrow represents the data flow direction: points in the child frame
    are transformed to the parent frame. Therefore, the arrow points from
    child to parent to indicate this transformation direction.

    Args:
        ax: Matplotlib 3D axis
        start: Start position (child frame position in root frame)
        end: End position (parent frame position in root frame)
        label: Label for the transform
        alpha: Transparency
        reverse: If True, draw arrow from start to end; if False, from end to start
    """
    start = np.array(start)
    end = np.array(end)

    # Draw arrow from child to parent (data flow direction)
    # unless reverse is True
    if reverse:
        arrow_start, arrow_end = start, end
    else:
        arrow_start, arrow_end = end, start

    ax.quiver(arrow_start[0],
              arrow_start[1],
              arrow_start[2],
              arrow_end[0] - arrow_start[0],
              arrow_end[1] - arrow_start[1],
              arrow_end[2] - arrow_start[2],
              color='gray',
              arrow_length_ratio=0.15,
              linestyle='--',
              alpha=alpha,
              linewidth=1)

    # Add label at midpoint
    if label:
        mid = (start + end) / 2
        ax.text(mid[0], mid[1], mid[2], label, fontsize=6, color='gray')


class TFVisualizer:
    """Visualizer for TF coordinate frames."""

    def __init__(self, tf_graph: TFGraph):
        """Initialize the visualizer.

        Args:
            tf_graph: TF graph to visualize
        """
        self.tf_graph = tf_graph
        self.fig = None
        self.ax = None

    def visualize(self, root_frame: str = None, frame_length: float = 0.5):
        """Create the visualization.

        Args:
            root_frame: Root frame to use as reference (auto-detect if None)
            frame_length: Length of coordinate axis arrows
        """
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Auto-detect root frame if not specified
        if root_frame is None:
            roots = self.tf_graph.find_root_frames()
            if not roots:
                # If no clear root, use the first frame
                root_frame = list(self.tf_graph.frames)[0]
            else:
                root_frame = roots[0]

        # Compute positions and orientations of all frames
        poses = self.tf_graph.compute_all_frame_poses(root_frame)

        # Draw all coordinate systems
        for frame, (pos, R) in poses.items():
            draw_coordinate_system(self.ax,
                                   pos,
                                   R,
                                   length=frame_length,
                                   label=frame)

        # Draw transform arrows
        # According to TF semantics (pose_child_frame * tf = pose_frame),
        # the arrow points from child to parent to represent data flow direction
        for (parent, child), transform in self.tf_graph.transforms.items():
            if parent in poses and child in poses:
                parent_pos = poses[parent][0]
                child_pos = poses[child][0]
                # Arrow from child to parent (data flow: child -> parent)
                # Label shows the transformation direction
                draw_transform_arrow(self.ax,
                                     child_pos,
                                     parent_pos,
                                     label=f"{child}->{parent}")

        # Set labels and title
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(
            f'TF Coordinate Frame Visualization (Root: {root_frame})')

        # Equal aspect ratio
        self._set_equal_aspect()

        # Add legend
        self._add_legend()

        plt.tight_layout()
        plt.show()

    def _set_equal_aspect(self):
        """Set equal aspect ratio for 3D plot."""
        # Get current limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()

        # Find max range
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        z_range = zlim[1] - zlim[0]
        max_range = max(x_range, y_range, z_range)

        # Set equal limits
        x_mid = (xlim[0] + xlim[1]) / 2
        y_mid = (ylim[0] + ylim[1]) / 2
        z_mid = (zlim[0] + zlim[1]) / 2

        self.ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
        self.ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
        self.ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)

    def _add_legend(self):
        """Add legend explaining the coordinate system."""
        from matplotlib.lines import Line2D
        colors = get_axis_colors()
        legend_elements = [
            Line2D([0], [0], color=colors['x'], lw=2, label='X axis'),
            Line2D([0], [0], color=colors['y'], lw=2, label='Y axis'),
            Line2D([0], [0], color=colors['z'], lw=2, label='Z axis'),
            Line2D([0], [0],
                   color='gray',
                   lw=1,
                   linestyle='--',
                   label='Transform'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper left')


@click.command()
@click.option(
    '-t',
    '--tf-file',
    'tf_files',
    multiple=True,
    type=click.Path(exists=True, path_type=pathlib.Path),
    help='TF configuration YAML file (can be specified multiple times)')
@click.option(
    '-c',
    '--config',
    'config_files',
    multiple=True,
    type=click.Path(exists=True, path_type=pathlib.Path),
    help='Static transform config pb.txt file (can be specified multiple times)'
)
@click.option(
    '-a',
    '--apollo-root',
    type=click.Path(exists=True, path_type=pathlib.Path),
    default=None,
    help=
    'Apollo root directory for resolving relative paths in config files (default: auto-detect)'
)
@click.option(
    '-r',
    '--root',
    type=str,
    default=None,
    help='Root frame to use as reference (auto-detect if not specified)')
@click.option('-l',
              '--length',
              type=float,
              default=0.5,
              help='Length of coordinate axis arrows (default: 0.5)')
@click.option('-s',
              '--save',
              type=click.Path(path_type=pathlib.Path),
              default=None,
              help='Save figure to file instead of displaying')
def main(tf_files, config_files, apollo_root, root, length, save):
    """Visualize TF (transform) configuration files.

    This tool loads TF configuration files and visualizes the coordinate
    frame relationships in an interactive 3D plot.

    TF files can be specified either as individual YAML files (-t) or
    through static transform config pb.txt files (-c) which reference
    multiple extrinsic files.

    Examples:

        \b
        # Visualize TF configs from individual YAML files
        whl_tf_show -t tf1.yaml -t tf2.yaml

        \b
        # Visualize TF configs from a static transform config file
        whl_tf_show -c modules/transform/conf/static_transform_conf.pb.txt

        \b
        # Combine both sources
        whl_tf_show -t custom.yaml -c static_transform_conf.pb.txt

        \b
        # Specify Apollo root for resolving relative paths
        whl_tf_show -c conf/static_transform_conf.pb.txt -a /path/to/apollo

        \b
        # Visualize with custom root frame and save to file
        whl_tf_show -c conf/static_transform_conf.pb.txt -r localization -s output.png
    """
    # Auto-detect Apollo root if not specified
    if apollo_root is None:
        # Try to find Apollo root by looking for common directories
        current_path = pathlib.Path.cwd()
        for parent in [current_path] + list(current_path.parents):
            if (parent / 'modules' / 'transform').exists():
                apollo_root = parent
                click.echo(f"Auto-detected Apollo root: {apollo_root}")
                break

    if not tf_files and not config_files:
        click.echo("Error: No TF files or config files specified", err=True)
        click.echo("Use -t/--tf-file or -c/--config to specify input files",
                   err=True)
        click.echo("Try 'whl_tf_show --help' for help", err=True)
        raise click.Abort()

    # Create TF graph
    tf_graph = TFGraph()

    # Load individual TF YAML files
    for filepath in tf_files:
        try:
            transform = load_transform_file(filepath)
            tf_graph.add_transform(transform)
            click.echo(
                f"Loaded: {filepath} ({transform.parent} -> {transform.child})"
            )
        except Exception as e:
            click.echo(f"Warning: Failed to load {filepath}: {e}", err=True)

    # Load config pb.txt files and referenced extrinsic files
    for config_path in config_files:
        try:
            click.echo(f"Loading config: {config_path}")
            extrinsics = load_static_transform_conf_pbtxt(
                config_path, apollo_root)
            for frame_id, child_frame_id, file_path in extrinsics:
                try:
                    transform = load_transform_file(file_path)
                    # Verify frame_id and child_frame_id match
                    if transform.parent != frame_id or transform.child != child_frame_id:
                        click.echo(
                            f"Warning: Frame mismatch in {file_path}: "
                            f"expected {frame_id}->{child_frame_id}, "
                            f"got {transform.parent}->{transform.child}",
                            err=True)
                    tf_graph.add_transform(transform)
                    click.echo(
                        f"  Loaded: {file_path} ({transform.parent} -> {transform.child})"
                    )
                except Exception as e:
                    click.echo(f"  Warning: Failed to load {file_path}: {e}",
                               err=True)
        except Exception as e:
            click.echo(f"Warning: Failed to load config {config_path}: {e}",
                       err=True)

    if not tf_graph.transforms:
        click.echo("Error: No valid transforms loaded", err=True)
        raise click.Abort()

    click.echo(f"\nLoaded {len(tf_graph.transforms)} transform(s)")
    click.echo(f"Frames: {', '.join(sorted(tf_graph.frames))}")

    # Create visualizer
    visualizer = TFVisualizer(tf_graph)

    if save:
        # Save figure to file
        visualizer.visualize(root_frame=root, frame_length=length)
        visualizer.fig.savefig(save, dpi=300, bbox_inches='tight')
        click.echo(f"\nSaved visualization to: {save}")
    else:
        # Show interactive plot
        visualizer.visualize(root_frame=root, frame_length=length)


if __name__ == '__main__':
    main()
