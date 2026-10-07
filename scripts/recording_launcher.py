#!/usr/bin/env python3

###############################################################################
# Copyright 2026 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""Launch one HMI-managed cyber recorder session.

This launcher is the HMI entrypoint for a single data-capture session. HMI
owns the process lifecycle; this program only selects storage, creates the
capture directory, and replaces itself with ``cyber_recorder record``.

Run only inside an Apollo container:

    python3 /apollo/scripts/recording_launcher.py \
        --config modules/dreamview/conf/recording/runtime.yaml

The configuration is passed unchanged to ``cyber_recorder`` and owns channel
selection and policy, including regex exclusions. The launcher creates:

    <storage>/data/records/capture-<UTC timestamp>/
      capture.json
      record*

Use ``--output-root`` only to select an explicit writable storage root. The
default selects internal NVMe first, then the largest writable /media mount,
and finally /apollo. Do not run this launcher in the background or use it to
stop a recorder; HMI owns those operations.
"""

import argparse
import datetime
import json
import os
import pathlib


def resolve_config_path(config_key):
    """Resolve a module config key using the WheelOS whole-file override."""
    key = pathlib.PurePosixPath(config_key)
    if key.is_absolute() or not key.parts or '..' in key.parts:
        raise ValueError('Config path must be a relative key: {}'.format(
            config_key))

    software_root = os.environ.get('APOLLO_ROOT_DIR', '/apollo')
    if not os.path.isabs(software_root) or not os.path.isdir(software_root):
        raise ValueError('Software root must be an accessible absolute '
                         'directory: {}'.format(software_root))
    software_root = os.path.realpath(software_root)
    default_path = os.path.join(software_root, *key.parts)

    config_root = os.environ.get('WHEELOS_CONFIG_ROOT')
    if config_root is not None:
        if not config_root or not os.path.isabs(config_root) or \
                not os.path.isdir(config_root):
            raise ValueError('WHEELOS_CONFIG_ROOT must be an accessible '
                             'absolute directory: {}'.format(config_root))
        config_root = os.path.realpath(config_root)
        override_path = os.path.join(config_root, *key.parts)
        current_path = config_root
        for part in key.parts:
            current_path = os.path.join(current_path, part)
            if os.path.islink(current_path):
                canonical_path = os.path.realpath(current_path)
                if os.path.commonpath((config_root, canonical_path)) != \
                        config_root or not os.path.exists(current_path):
                    raise ValueError('Config override contains a broken or '
                                     'escaping symlink: {}'.format(
                                         current_path))
            if not os.path.lexists(current_path):
                break
        if os.path.lexists(override_path):
            selected_path = os.path.realpath(override_path)
            if os.path.commonpath((config_root, selected_path)) != config_root:
                raise ValueError('Config override escapes WHEELOS_CONFIG_ROOT: '
                                 '{}'.format(override_path))
            if not os.path.isfile(selected_path) or \
                    not os.access(selected_path, os.R_OK):
                raise ValueError('Config override is not a readable file: '
                                 '{}'.format(override_path))
            source = 'override'
        else:
            selected_path = default_path
            source = 'default'
    else:
        selected_path = default_path
        source = 'default'

    if not os.path.isfile(selected_path) or \
            not os.access(selected_path, os.R_OK):
        raise ValueError('Config file is not a readable file: {}'.format(
            selected_path))
    selected_path = os.path.realpath(selected_path)
    if os.path.commonpath((software_root, selected_path)) != software_root \
            and source == 'default':
        raise ValueError('Default config escapes software root: {}'.format(
            selected_path))
    print('[CONFIG] {}: {} {}'.format(key.as_posix(), source, selected_path))
    return selected_path


class RecordingStorageResolver(object):
    """Select one writable recording root."""

    def resolve(self, output_root=None):
        """Return an explicit root or the highest-priority mounted storage."""
        if output_root:
            return output_root

        import psutil

        candidates = []
        for partition in psutil.disk_partitions():
            mountpoint = partition.mountpoint
            if not mountpoint.startswith('/media/'):
                continue
            if not os.access(mountpoint, os.W_OK):
                continue
            candidates.append((
                mountpoint.startswith('/media/apollo/internal_nvme'),
                self.available_bytes(mountpoint),
                mountpoint,
            ))

        if candidates:
            return max(candidates)[2]
        return '/apollo'

    @staticmethod
    def available_bytes(path):
        """Return available bytes for a mounted path."""
        stat = os.statvfs(path)
        return stat.f_frsize * stat.f_bavail


def parse_args():
    """Parse launcher arguments."""
    parser = argparse.ArgumentParser(
        description='Launch one HMI-managed cyber_recorder session.',
        epilog=('HMI owns start and stop. The YAML config owns channel '
                'selection, including regex exclusions.'))
    parser.add_argument(
        '--config', required=True,
        help='Required cyber_recorder YAML configuration.')
    parser.add_argument(
        '--output-root',
        help='Writable recording root; defaults to the best mounted disk.')
    return parser.parse_args()


def create_session(output_root, config_path):
    """Create and describe one recording session directory."""
    capture_id = datetime.datetime.utcnow().strftime('capture-%Y%m%dT%H%M%SZ')
    session_dir = os.path.join(output_root, 'data', 'records', capture_id)
    os.makedirs(session_dir)

    metadata_path = os.path.join(session_dir, 'capture.json')
    with open(metadata_path, 'w') as metadata_file:
        json.dump({
            'config': config_path,
            'created_at_utc': datetime.datetime.utcnow().strftime(
                '%Y-%m-%dT%H:%M:%SZ'),
        }, metadata_file, indent=2, sort_keys=True)
        metadata_file.write('\n')

    return session_dir


def main():
    """Create a session and replace this process with cyber_recorder."""
    args = parse_args()
    config_path = resolve_config_path(args.config)

    output_root = RecordingStorageResolver().resolve(args.output_root)
    session_dir = create_session(output_root, config_path)
    output_path = os.path.join(session_dir, 'record')
    print('Recording to {}'.format(session_dir))

    os.execv('/bin/bash', [
        'bash', '-lc',
        'source /apollo/scripts/apollo_base.sh && '
        'source /apollo/scripts/runtime_env.sh && '
        'exec cyber_recorder record --config "$1" --output "$2"',
        'recording_launcher', config_path, output_path,
    ])


if __name__ == '__main__':
    main()
