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
"""
whl-tf-trans - Coordinate Transform Tool for Apollo-Lite

Transforms target point coordinates from global frame to local vehicle frame where:
- Origin: current vehicle position
- Y-axis: vehicle heading direction (forward)
- X-axis: perpendicular to heading (pointing left)

Usage:
    ./whl_tf_trans.py --curr-x <x> --curr-y <y> --curr-heading <h> -x <tx> -y <ty> --heading <th>

Example:
    ./whl_tf_trans.py --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x 272091.94 -y 4020872.85 --heading 3.086
"""

import sys
import math
import click


def angle_diff(from_angle, to_angle):
    """Compute angle difference from 'to' to 'from', normalized to [-PI, PI]."""
    return math.atan2(math.sin(from_angle - to_angle), math.cos(from_angle - to_angle))


def transform_to_local_frame(curr_x, curr_y, curr_heading,
                             target_x, target_y, target_heading):
    """
    Transform target point to local vehicle frame.

    Args:
        curr_x, curr_y: Current vehicle position in global frame
        curr_heading: Current vehicle heading in radians (angle from global X axis, CCW positive)
        target_x, target_y: Target position in global frame
        target_heading: Target heading in radians

    Returns:
        Tuple of (local_x, local_y, local_heading) in vehicle frame
        - local_x: positive = left, negative = right
        - local_y: positive = forward, negative = backward
        - local_heading: angle relative to vehicle heading (CCW positive)
    """
    # 1. Translation: subtract current position
    dx = target_x - curr_x
    dy = target_y - curr_y

    # 2. Rotation: align current heading with Y-axis
    # Rotation angle: (PI/2 - curr_heading)
    # This makes the vehicle heading point to +Y direction
    local_x = dx * math.sin(curr_heading) - dy * math.cos(curr_heading)
    local_y = dx * math.cos(curr_heading) + dy * math.sin(curr_heading)

    # 3. Heading difference (normalized to [-PI, PI])
    local_heading = angle_diff(target_heading, curr_heading)

    return local_x, local_y, local_heading


def transform_to_global_frame(curr_x, curr_y, curr_heading,
                              local_x, local_y, local_heading):
    """
    Transform local coordinates back to global frame.

    Args:
        curr_x, curr_y: Current vehicle position in global frame
        curr_heading: Current vehicle heading in radians
        local_x, local_y: Target position in local vehicle frame
        local_heading: Target heading in local frame (relative to vehicle)

    Returns:
        Tuple of (global_x, global_y, global_heading)
    """
    # Inverse rotation: transpose of forward rotation matrix
    # Forward: [local_x]   [sin(θ)  -cos(θ)] [dx]
    #          [local_y] = [cos(θ)   sin(θ)] [dy]
    # Inverse: [dx]   [sin(θ)  cos(θ)] [local_x]
    #          [dy] = [-cos(θ) sin(θ)] [local_y]

    global_x = curr_x + local_x * math.sin(curr_heading) + local_y * math.cos(curr_heading)
    global_y = curr_y - local_x * math.cos(curr_heading) + local_y * math.sin(curr_heading)

    # Add current heading to relative heading
    global_heading = curr_heading + local_heading
    # Normalize to [-PI, PI]
    global_heading = math.atan2(math.sin(global_heading), math.cos(global_heading))

    return global_x, global_y, global_heading


def format_output_verbose(local_x, local_y, local_heading):
    """Format the verbose output for display."""
    click.echo("=" * 60)
    click.echo("Local Frame Result (Vehicle Local Frame)")
    click.echo("=" * 60)
    click.echo(f"  X coordinate (Left+ Right-):  {local_x:10.4f} m")
    click.echo(f"  Y coordinate (Front+ Back-):  {local_y:10.4f} m")
    click.echo(f"  Heading (relative):            {local_heading:10.4f} rad = {math.degrees(local_heading):8.2f}°")
    click.echo("=" * 60)


def format_output_global_verbose(global_x, global_y, global_heading):
    """Format the verbose output for global frame."""
    click.echo("=" * 60)
    click.echo("Global Frame Result")
    click.echo("=" * 60)
    click.echo(f"  X coordinate:          {global_x:10.4f} m")
    click.echo(f"  Y coordinate:          {global_y:10.4f} m")
    click.echo(f"  Heading:               {global_heading:10.4f} rad = {math.degrees(global_heading):8.2f}°")
    click.echo("=" * 60)


@click.command()
@click.option('--curr-x', type=float, required=True, help='Current vehicle X position')
@click.option('--curr-y', type=float, required=True, help='Current vehicle Y position')
@click.option('--curr-heading', type=float, required=True, help='Current vehicle heading (radians)')
@click.option('-x', '--target-x', type=float, required=True, help='Target X position')
@click.option('-y', '--target-y', type=float, required=True, help='Target Y position')
@click.option('--heading', type=float, required=True, help='Target heading (radians)')
@click.option('-i', '--inverse', is_flag=True,
              help='Inverse transformation: local -> global')
@click.option('--heading-unit', type=click.Choice(['rad', 'deg']), default='rad',
              help='Heading unit in output (default: rad)')
@click.option('--porcelain', is_flag=True,
              help='Porcelain mode: suppress all log output, only print CSV data')
@click.option('--output-format', type=click.Choice(['text', 'csv']), default='text',
              help='Output format (default: text)')
@click.option('--separator', default=',',
              help='Separator for CSV output (default: ",")')
def main(curr_x, curr_y, curr_heading, target_x, target_y, heading, inverse,
         heading_unit, porcelain, output_format, separator):
    """
    Transform coordinates between global and local vehicle frames.

    \b
    Examples:
      # Transform global to local (verbose text output, heading in radians)
      python whl_tf_trans.py --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x 272091.94 -y 4020872.85 --heading 3.086

      # Transform global to local (heading in degrees)
      python whl_tf_trans.py --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x 272091.94 -y 4020872.85 --heading 3.086 --heading-unit deg

      # CSV output with heading in degrees
      python whl_tf_trans.py --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x 272091.94 -y 4020872.85 --heading 3.086 --output-format csv --heading-unit deg

      # Porcelain mode (no header, heading in degrees)
      python whl_tf_trans.py --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x 272091.94 -y 4020872.85 --heading 3.086 --porcelain --heading-unit deg

      # Inverse transformation (local to global) - handles negative values correctly
      python whl_tf_trans.py --inverse --curr-x 272115.38 --curr-y 4020849.63 --curr-heading 1.57 -x -23.44 -y 23.22 --heading 1.52
    """

    verbose = not porcelain and output_format == 'text'

    if inverse:
        # Local to Global
        global_x, global_y, global_heading = transform_to_global_frame(
            curr_x, curr_y, curr_heading, target_x, target_y, heading
        )
        if verbose:
            format_output_global_verbose(global_x, global_y, global_heading)

        # Convert heading to degrees if needed
        heading_output = math.degrees(global_heading) if heading_unit == 'deg' else global_heading

        # CSV output
        if output_format == 'csv':
            if not porcelain:
                unit_suffix = '_deg' if heading_unit == 'deg' else '_rad'
                click.echo(separator.join(['x', 'y', f'heading{unit_suffix}']))
            click.echo(separator.join([
                f"{global_x:.4f}",
                f"{global_y:.4f}",
                f"{heading_output:.4f}"
            ]))
        elif porcelain:
            click.echo(f"{global_x:.4f},{global_y:.4f},{heading_output:.4f}")
    else:
        # Global to Local
        local_x, local_y, local_heading = transform_to_local_frame(
            curr_x, curr_y, curr_heading, target_x, target_y, heading
        )
        if verbose:
            format_output_verbose(local_x, local_y, local_heading)

        # Convert heading to degrees if needed
        heading_output = math.degrees(local_heading) if heading_unit == 'deg' else local_heading

        # CSV output
        if output_format == 'csv':
            if not porcelain:
                unit_suffix = '_deg' if heading_unit == 'deg' else '_rad'
                click.echo(separator.join(['x', 'y', f'heading{unit_suffix}']))
            click.echo(separator.join([
                f"{local_x:.4f}",
                f"{local_y:.4f}",
                f"{heading_output:.4f}"
            ]))
        elif porcelain:
            click.echo(f"{local_x:.4f},{local_y:.4f},{heading_output:.4f}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
