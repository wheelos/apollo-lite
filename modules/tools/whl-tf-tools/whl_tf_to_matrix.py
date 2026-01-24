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
"""whl-tf-to-matrix

Convert TF (transform) configuration to 4x4 homogeneous transformation matrix.
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


def tf_to_matrix(translation: tuple, rotation: tuple) -> np.ndarray:
    """Convert TF translation and rotation to 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_rotation_matrix(rotation)
    T[:3, 3] = translation
    return T


def format_matrix(matrix: np.ndarray, precision: int = 6) -> str:
    """Format matrix for display."""
    lines = []
    for row in matrix:
        row_str = "  ".join([f"{val:.{precision}f}" for val in row])
        lines.append(f"[ {row_str} ]")
    return "\n".join(lines)


def format_matrix_python(matrix: np.ndarray, precision: int = 6) -> str:
    """Format matrix as Python code."""
    lines = ["T = np.array(["]
    for row in matrix:
        row_str = ", ".join([f"{val:.{precision}f}" for val in row])
        lines.append(f"    [{row_str}],")
    lines.append("])")
    return "\n".join(lines)


def format_matrix_cpp(matrix: np.ndarray, precision: int = 6) -> str:
    """Format matrix as C++ Eigen code."""
    cpp_header = "Eigen::Matrix4d T;"
    lines = [cpp_header]
    for i in range(4):
        for j in range(4):
            lines.append(f"T({i},{j}) = {matrix[i, j]:.{precision}f};")
    return "\n".join(lines)


def load_transform_file(filepath: pathlib.Path) -> tuple:
    """Load transform from a YAML file."""
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

    return translation, rotation, parent, child


@click.group()
def main():
    """Convert TF (transform) configuration to 4x4 transformation matrix."""
    pass


@main.command('file')
@click.argument('filepath', type=click.Path(exists=True, path_type=pathlib.Path))
@click.option('-f', '--format', 'output_format',
              type=click.Choice(['table', 'python', 'cpp'], case_sensitive=False),
              default='table', help='Output format (default: table)')
@click.option('-p', '--precision', type=int, default=6,
              help='Decimal precision (default: 6)')
def from_file(filepath, output_format, precision):
    """Convert transform from a YAML file."""
    try:
        translation, rotation, parent, child = load_transform_file(filepath)
    except Exception as e:
        click.echo(f"Error loading file: {e}", err=True)
        sys.exit(1)

    matrix = tf_to_matrix(translation, rotation)

    click.echo(f"\nTransform: {parent} -> {child}")
    click.echo(f"Translation: {translation}")
    click.echo(f"Rotation (quaternion): {rotation}")
    click.echo("\n4x4 Transformation Matrix:")

    if output_format == 'table':
        click.echo(format_matrix(matrix, precision))
    elif output_format == 'python':
        click.echo(format_matrix_python(matrix, precision))
    elif output_format == 'cpp':
        click.echo(format_matrix_cpp(matrix, precision))


@main.command('cli')
@click.option('--tx', type=float, required=True, help='Translation X (meters)')
@click.option('--ty', type=float, required=True, help='Translation Y (meters)')
@click.option('--tz', type=float, required=True, help='Translation Z (meters)')
@click.option('--qx', type=float, required=True, help='Quaternion X')
@click.option('--qy', type=float, required=True, help='Quaternion Y')
@click.option('--qz', type=float, required=True, help='Quaternion Z')
@click.option('--qw', type=float, required=True, help='Quaternion W')
@click.option('-f', '--format', 'output_format',
              type=click.Choice(['table', 'python', 'cpp'], case_sensitive=False),
              default='table', help='Output format (default: table)')
@click.option('-p', '--precision', type=int, default=6,
              help='Decimal precision (default: 6)')
def from_cli(tx, ty, tz, qx, qy, qz, qw, output_format, precision):
    """Convert transform from command line arguments."""
    translation = (tx, ty, tz)
    rotation = (qx, qy, qz, qw)

    matrix = tf_to_matrix(translation, rotation)

    click.echo(f"\nTranslation: ({tx}, {ty}, {tz})")
    click.echo(f"Rotation (quaternion): ({qx}, {qy}, {qz}, {qw})")
    click.echo("\n4x4 Transformation Matrix:")

    if output_format == 'table':
        click.echo(format_matrix(matrix, precision))
    elif output_format == 'python':
        click.echo(format_matrix_python(matrix, precision))
    elif output_format == 'cpp':
        click.echo(format_matrix_cpp(matrix, precision))


@main.command('euler')
@click.option('--tx', type=float, required=True, help='Translation X (meters)')
@click.option('--ty', type=float, required=True, help='Translation Y (meters)')
@click.option('--tz', type=float, required=True, help='Translation Z (meters)')
@click.option('--roll', type=float, required=True, help='Roll rotation (radians)')
@click.option('--pitch', type=float, required=True, help='Pitch rotation (radians)')
@click.option('--yaw', type=float, required=True, help='Yaw rotation (radians)')
@click.option('-f', '--format', 'output_format',
              type=click.Choice(['table', 'python', 'cpp'], case_sensitive=False),
              default='table', help='Output format (default: table)')
@click.option('-p', '--precision', type=int, default=6,
              help='Decimal precision (default: 6)')
@click.option('--degrees', is_flag=True,
              help='Input angles in degrees instead of radians')
def from_euler(tx, ty, tz, roll, pitch, yaw, output_format, precision, degrees):
    """Convert transform from Euler angles (roll, pitch, yaw)."""
    from scipy.spatial.transform import Rotation

    if degrees:
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

    r = Rotation.from_euler('xyz', [roll, pitch, yaw])
    qx, qy, qz, qw = r.as_quat()

    translation = (tx, ty, tz)
    rotation = (qx, qy, qz, qw)

    matrix = tf_to_matrix(translation, rotation)

    if degrees:
        roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
        click.echo(f"\nTranslation: ({tx}, {ty}, {tz})")
        click.echo(f"Rotation (Euler, degrees): roll={roll_deg}, pitch={pitch_deg}, yaw={yaw_deg}")
    else:
        click.echo(f"\nTranslation: ({tx}, {ty}, {tz})")
        click.echo(f"Rotation (Euler, radians): roll={roll}, pitch={pitch}, yaw={yaw}")

    click.echo(f"Rotation (quaternion): ({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})")
    click.echo("\n4x4 Transformation Matrix:")

    if output_format == 'table':
        click.echo(format_matrix(matrix, precision))
    elif output_format == 'python':
        click.echo(format_matrix_python(matrix, precision))
    elif output_format == 'cpp':
        click.echo(format_matrix_cpp(matrix, precision))


if __name__ == '__main__':
    main()
