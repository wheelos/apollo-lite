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
"""whl-tf-query

Interactive TF query tool.
Load multiple TF configuration files and query transforms between any two frames.
"""

import pathlib
import sys

import click
import numpy as np
import yaml


def quaternion_to_rotation_matrix(q: tuple) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
    x, y, z, w = q
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    R = np.array(
        [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R


class Transform:
    """Represents a transform between parent and child frames."""

    def __init__(self, parent: str, child: str,
                 translation: tuple, rotation: tuple):
        """Initialize a Transform."""
        self.parent = parent
        self.child = child
        self.translation = np.array(translation)
        self.rotation = rotation
        self.rotation_matrix = quaternion_to_rotation_matrix(rotation)

    def get_transform_matrix(self) -> np.ndarray:
        """Get 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = self.translation
        return T

    def get_inverse_matrix(self) -> np.ndarray:
        """Get inverse 4x4 transformation matrix."""
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
        self.transforms = {}  # (parent, child) -> Transform
        self.frames = set()

    def add_transform(self, transform: Transform):
        """Add a transform to the graph."""
        key = (transform.parent, transform.child)
        self.transforms[key] = transform
        self.frames.add(transform.parent)
        self.frames.add(transform.child)

    def compute_transform(self, from_frame: str, to_frame: str,
                         visited: set = None) -> tuple:
        """Compute transform from to_frame to from_frame.

        Args:
            from_frame: Target frame (result frame)
            to_frame: Source frame (input frame)
            visited: Set of visited frames (for cycle detection)

        Returns:
            Tuple of (4x4 transformation matrix, path list)
        """
        if visited is None:
            visited = set()

        if from_frame == to_frame:
            return np.eye(4), [from_frame]

        if from_frame in visited:
            raise ValueError(f"Cycle detected in transform graph")

        visited.add(from_frame)

        # Forward: find transforms where from_frame is the parent (parent -> child)
        for (parent, child), transform in self.transforms.items():
            if parent == from_frame:
                try:
                    child_transform, path = self.compute_transform(
                        child, to_frame, visited.copy())
                    return (transform.get_transform_matrix() @ child_transform,
                           [from_frame] + path)
                except ValueError:
                    continue

        # Backward: find transforms where from_frame is the child (child <- parent)
        # Use inverse transform to go from child to parent
        for (parent, child), transform in self.transforms.items():
            if child == from_frame:
                try:
                    parent_transform, path = self.compute_transform(
                        parent, to_frame, visited.copy())
                    return (transform.get_inverse_matrix() @ parent_transform,
                           [from_frame] + path)
                except ValueError:
                    continue

        raise ValueError(
            f"No transform path found from '{to_frame}' to '{from_frame}'")

    def list_frames(self) -> list:
        """Return sorted list of all frame IDs."""
        return sorted(self.frames)

    def list_transforms(self) -> list:
        """Return list of all transforms as (parent, child) tuples."""
        return list(self.transforms.keys())


def load_transform_file(filepath: pathlib.Path) -> Transform:
    """Load a transform from a YAML file."""
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


def load_static_transform_conf_pbtxt(
        filepath: pathlib.Path,
        apollo_root: pathlib.Path = None
) -> list:
    """Load transform configuration from a pb.txt file."""
    with open(filepath, 'r') as f:
        content = f.read()

    results = []
    lines = content.split('\n')
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith('extrinsic_file') and '{' in line:
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
                full_path = pathlib.Path(file_path)
                if not full_path.is_absolute() and apollo_root:
                    full_path = apollo_root / full_path.relative_to('/apollo')
                results.append((frame_id, child_frame_id, full_path))

        i += 1

    return results


def matrix_to_quaternion(T: np.ndarray) -> tuple:
    """Convert 4x4 transformation matrix to translation and quaternion."""
    from scipy.spatial.transform import Rotation

    translation = tuple(T[:3, 3])
    R = T[:3, :3]
    r = Rotation.from_matrix(R)
    quaternion = tuple(r.as_quat())
    return translation, quaternion


def format_transform_yaml(frame_id: str, child_frame_id: str,
                         translation: tuple, rotation: tuple) -> str:
    """Format transform as YAML string."""
    return f"""# proj: +proj=utm +zone=51 +ellps=WGS84
# scale:1.11177
header:
  stamp:
    secs: 1422601952
    nsecs: 288805456
  seq: 0
  frame_id: {frame_id}
transform:
  translation:
    x: {translation[0]:.5f}
    y: {translation[1]:.5f}
    z: {translation[2]:.5f}
  rotation:
    x: {rotation[0]:.8f}
    y: {rotation[1]:.8f}
    z: {rotation[2]:.8f}
    w: {rotation[3]:.8f}
child_frame_id: {child_frame_id}
"""


def format_matrix(T: np.ndarray, precision: int = 6) -> str:
    """Format matrix for display."""
    lines = []
    for row in T:
        row_str = "  ".join([f"{val:.{precision}f}" for val in row])
        lines.append(f"[ {row_str} ]")
    return "\n".join(lines)


@click.group()
def main():
    """Interactive TF query tool - load configs and query transforms."""
    pass


@main.command('load')
@click.argument('tf_files', nargs=-1,
                type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-c', '--config', 'config_files',
              multiple=True, type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-a', '--apollo-root',
              type=click.Path(exists=True, path_type=pathlib.Path), default=None)
@click.option('-o', '--output', type=click.Path(path_type=pathlib.Path),
              help='Save loaded graph to file (for later use with --graph-file)')
@click.option('--list', 'list_frames', is_flag=True,
              help='List all loaded frames')
def load_configs(tf_files, config_files, apollo_root, output, list_frames):
    """Load TF configuration files and display loaded frames.

    \b
    Examples:
        # Load individual TF files
        whl_tf_query load -t tf1.yaml -t tf2.yaml

        # Load from static transform config
        whl_tf_query load -c modules/transform/conf/static_transform_conf.pb.txt

        # Combine both
        whl_tf_query load -t custom.yaml -c static_transform_conf.pb.txt
    """
    # Auto-detect Apollo root if not specified
    if apollo_root is None:
        current_path = pathlib.Path.cwd()
        for parent in [current_path] + list(current_path.parents):
            if (parent / 'modules' / 'transform').exists():
                apollo_root = parent
                click.echo(f"Auto-detected Apollo root: {apollo_root}")
                break

    if not tf_files and not config_files:
        click.echo("Error: No files specified", err=True)
        click.echo("Use TF_FILES arguments or -c/--config option", err=True)
        raise click.Abort()

    tf_graph = TFGraph()

    # Load individual TF YAML files
    for filepath in tf_files:
        try:
            transform = load_transform_file(filepath)
            tf_graph.add_transform(transform)
            click.echo(f"Loaded: {filepath.name} ({transform.parent} -> {transform.child})")
        except Exception as e:
            click.echo(f"Warning: Failed to load {filepath}: {e}", err=True)

    # Load config pb.txt files
    for config_path in config_files:
        try:
            click.echo(f"Loading config: {config_path}")
            extrinsics = load_static_transform_conf_pbtxt(config_path, apollo_root)
            for frame_id, child_frame_id, file_path in extrinsics:
                try:
                    transform = load_transform_file(file_path)
                    if transform.parent != frame_id or transform.child != child_frame_id:
                        click.echo(
                            f"  Warning: Frame mismatch in {file_path}: "
                            f"expected {frame_id}->{child_frame_id}, "
                            f"got {transform.parent}->{transform.child}", err=True)
                    tf_graph.add_transform(transform)
                    click.echo(f"  Loaded: {file_path.name} ({transform.parent} -> {transform.child})")
                except Exception as e:
                    click.echo(f"  Warning: Failed to load {file_path}: {e}", err=True)
        except Exception as e:
            click.echo(f"Warning: Failed to load config {config_path}: {e}", err=True)

    if not tf_graph.transforms:
        click.echo("Error: No valid transforms loaded", err=True)
        raise click.Abort()

    click.echo(f"\nLoaded {len(tf_graph.transforms)} transform(s)")
    click.echo(f"Frames: {', '.join(tf_graph.list_frames())}")

    if list_frames:
        click.echo("\nAvailable transforms:")
        for parent, child in tf_graph.list_transforms():
            click.echo(f"  {parent} -> {child}")

    if output:
        import pickle
        with open(output, 'wb') as f:
            pickle.dump(tf_graph, f)
        click.echo(f"\nGraph saved to: {output}")


@main.command('query')
@click.option('-t', '--tf-file', 'tf_files', multiple=True,
              type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-c', '--config', 'config_files', multiple=True,
              type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-a', '--apollo-root',
              type=click.Path(exists=True, path_type=pathlib.Path), default=None)
@click.option('-g', '--graph-file', type=click.Path(exists=True, path_type=pathlib.Path),
              help='Load previously saved graph')
@click.argument('from_frame', type=str)
@click.argument('to_frame', type=str)
@click.option('-o', '--output', type=click.Path(path_type=pathlib.Path),
              help='Output transform to YAML file')
@click.option('-f', '--format', 'output_format',
              type=click.Choice(['yaml', 'matrix', 'all'], case_sensitive=False),
              default='yaml', help='Output format')
@click.option('-p', '--precision', type=int, default=6,
              help='Decimal precision for matrix output')
def query_transform(tf_files, config_files, apollo_root, graph_file,
                   from_frame, to_frame, output, output_format, precision):
    """Query transform between two frames.

    \b
    Examples:
        # Query transform with direct files
        whl_tf_query query -t tf1.yaml -t tf2.yaml imu rslidar_main_front

        # Query transform from config
        whl_tf_query query -c static_transform_conf.pb.txt rslidar_main_front rslidar_main_rear

        # Query with saved graph
        whl_tf_query query -g graph.pkl imu localization -o transform.yaml

        # Output as matrix
        whl_tf_query query -t tf1.yaml -t tf2.yaml imu rslidar -f matrix
    """
    tf_graph = None

    # Load from saved graph if provided
    if graph_file:
        import pickle
        with open(graph_file, 'rb') as f:
            tf_graph = pickle.load(f)
        click.echo(f"Loaded graph from: {graph_file}")

    # Otherwise load from files
    if tf_graph is None:
        # Auto-detect Apollo root if not specified
        if apollo_root is None:
            current_path = pathlib.Path.cwd()
            for parent in [current_path] + list(current_path.parents):
                if (parent / 'modules' / 'transform').exists():
                    apollo_root = parent
                    break

        if not tf_files and not config_files:
            click.echo("Error: No input files specified", err=True)
            click.echo("Use -t/--tf-file, -c/--config, or -g/--graph-file", err=True)
            raise click.Abort()

        tf_graph = TFGraph()

        # Load individual TF YAML files
        for filepath in tf_files:
            try:
                transform = load_transform_file(filepath)
                tf_graph.add_transform(transform)
            except Exception as e:
                click.echo(f"Warning: Failed to load {filepath}: {e}", err=True)

        # Load config pb.txt files
        for config_path in config_files:
            try:
                extrinsics = load_static_transform_conf_pbtxt(config_path, apollo_root)
                for frame_id, child_frame_id, file_path in extrinsics:
                    try:
                        transform = load_transform_file(file_path)
                        tf_graph.add_transform(transform)
                    except Exception:
                        pass
            except Exception as e:
                click.echo(f"Warning: Failed to load config {config_path}: {e}", err=True)

    # Check if frames exist
    if from_frame not in tf_graph.frames:
        click.echo(f"Error: Frame '{from_frame}' not found in loaded transforms", err=True)
        click.echo(f"Available frames: {', '.join(tf_graph.list_frames())}", err=True)
        raise click.Abort()

    if to_frame not in tf_graph.frames:
        click.echo(f"Error: Frame '{to_frame}' not found in loaded transforms", err=True)
        click.echo(f"Available frames: {', '.join(tf_graph.list_frames())}", err=True)
        raise click.Abort()

    # Compute transform
    try:
        T, path = tf_graph.compute_transform(from_frame, to_frame)
    except ValueError as e:
        click.echo(f"Error: {e}", err=True)
        raise click.Abort()

    # Convert to translation and quaternion
    translation, rotation = matrix_to_quaternion(T)

    click.echo(f"\nTransform: {to_frame} -> {from_frame}")
    click.echo(f"Path: {' -> '.join(path)}")
    click.echo()

    if output_format in ('yaml', 'all'):
        yaml_content = format_transform_yaml(from_frame, to_frame, translation, rotation)
        click.echo("=" * 60)
        click.echo(yaml_content)
        click.echo("=" * 60)

    if output_format in ('matrix', 'all'):
        if output_format == 'all':
            click.echo()
        click.echo("4x4 Transformation Matrix:")
        click.echo(format_matrix(T, precision))
        click.echo()
        click.echo(f"Translation: ({translation[0]:.5f}, {translation[1]:.5f}, {translation[2]:.5f})")
        click.echo(f"Quaternion: ({rotation[0]:.8f}, {rotation[1]:.8f}, {rotation[2]:.8f}, {rotation[3]:.8f})")

    if output:
        yaml_content = format_transform_yaml(from_frame, to_frame, translation, rotation)
        with open(output, 'w') as f:
            f.write(yaml_content)
        click.echo(f"\nTransform saved to: {output}")


@main.command('interactive')
@click.option('-t', '--tf-file', 'tf_files', multiple=True,
              type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-c', '--config', 'config_files', multiple=True,
              type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-a', '--apollo-root',
              type=click.Path(exists=True, path_type=pathlib.Path), default=None)
@click.option('-g', '--graph-file', type=click.Path(exists=True, path_type=pathlib.Path),
              help='Load previously saved graph')
def interactive_mode(tf_files, config_files, apollo_root, graph_file):
    """Interactive mode for querying multiple transforms.

    \b
    Examples:
        # Start interactive mode with files
        whl_tf_query interactive -t tf1.yaml -t tf2.yaml

        # Start with saved graph
        whl_tf_query interactive -g graph.pkl
    """
    tf_graph = None

    # Load from saved graph if provided
    if graph_file:
        import pickle
        with open(graph_file, 'rb') as f:
            tf_graph = pickle.load(f)
        click.echo(f"Loaded graph from: {graph_file}")

    # Otherwise load from files
    if tf_graph is None:
        # Auto-detect Apollo root
        if apollo_root is None:
            current_path = pathlib.Path.cwd()
            for parent in [current_path] + list(current_path.parents):
                if (parent / 'modules' / 'transform').exists():
                    apollo_root = parent
                    break

        if not tf_files and not config_files:
            click.echo("Error: No input files specified", err=True)
            click.echo("Use -t/--tf-file, -c/--config, or -g/--graph-file", err=True)
            raise click.Abort()

        tf_graph = TFGraph()

        # Load files
        for filepath in tf_files:
            try:
                transform = load_transform_file(filepath)
                tf_graph.add_transform(transform)
                click.echo(f"Loaded: {filepath.name}")
            except Exception as e:
                click.echo(f"Warning: Failed to load {filepath}: {e}", err=True)

        for config_path in config_files:
            try:
                extrinsics = load_static_transform_conf_pbtxt(config_path, apollo_root)
                for frame_id, child_frame_id, file_path in extrinsics:
                    try:
                        transform = load_transform_file(file_path)
                        tf_graph.add_transform(transform)
                    except Exception:
                        pass
            except Exception as e:
                click.echo(f"Warning: Failed to load config {config_path}: {e}", err=True)

    click.echo(f"\nLoaded {len(tf_graph.transforms)} transform(s)")
    click.echo(f"Available frames: {', '.join(tf_graph.list_frames())}")
    click.echo("\nCommands: 'query FROM TO', 'list', 'help', 'quit'")

    while True:
        try:
            user_input = input("\n> ").strip()

            if not user_input:
                continue

            if user_input in ('quit', 'exit', 'q'):
                click.echo("Goodbye!")
                break

            if user_input in ('help', 'h', '?'):
                click.echo("\nCommands:")
                click.echo("  query FROM TO    - Query transform FROM <- TO")
                click.echo("  list             - List all frames and transforms")
                click.echo("  help             - Show this help")
                click.echo("  quit             - Exit")
                continue

            if user_input == 'list':
                click.echo("\nAvailable frames:")
                for frame in tf_graph.list_frames():
                    click.echo(f"  {frame}")
                click.echo("\nAvailable transforms:")
                for parent, child in tf_graph.list_transforms():
                    click.echo(f"  {parent} -> {child}")
                continue

            if user_input.startswith('query '):
                parts = user_input.split()
                if len(parts) != 3:
                    click.echo("Usage: query FROM TO")
                    continue

                from_frame, to_frame = parts[1], parts[2]

                if from_frame not in tf_graph.frames:
                    click.echo(f"Error: Frame '{from_frame}' not found")
                    continue
                if to_frame not in tf_graph.frames:
                    click.echo(f"Error: Frame '{to_frame}' not found")
                    continue

                try:
                    T, path = tf_graph.compute_transform(from_frame, to_frame)
                    translation, rotation = matrix_to_quaternion(T)

                    click.echo(f"\nTransform: {to_frame} -> {from_frame}")
                    click.echo(f"Path: {' -> '.join(path)}")
                    click.echo(f"Translation: ({translation[0]:.5f}, {translation[1]:.5f}, {translation[2]:.5f})")
                    click.echo(f"Quaternion (xyzw): ({rotation[0]:.8f}, {rotation[1]:.8f}, {rotation[2]:.8f}, {rotation[3]:.8f})")
                except ValueError as e:
                    click.echo(f"Error: {e}")
                continue

            click.echo(f"Unknown command: {user_input}")
            click.echo("Type 'help' for available commands")

        except (EOFError, KeyboardInterrupt):
            click.echo("\nGoodbye!")
            break
        except Exception as e:
            click.echo(f"Error: {e}")


if __name__ == '__main__':
    main()
