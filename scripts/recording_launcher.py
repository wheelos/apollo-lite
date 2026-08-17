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
        --config /apollo/modules/dreamview/conf/recording/runtime.yaml

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
    if not os.path.isfile(args.config):
        raise ValueError('Recorder configuration does not exist: {}'.format(
            args.config))

    output_root = RecordingStorageResolver().resolve(args.output_root)
    session_dir = create_session(output_root, args.config)
    output_path = os.path.join(session_dir, 'record')
    print('Recording to {}'.format(session_dir))

    os.execv('/bin/bash', [
        'bash', '-lc',
        'source /apollo/scripts/apollo_base.sh && '
        'source /apollo/scripts/runtime_env.sh && '
        'exec cyber_recorder record --config "$1" --output "$2"',
        'recording_launcher', args.config, output_path,
    ])


if __name__ == '__main__':
    main()
