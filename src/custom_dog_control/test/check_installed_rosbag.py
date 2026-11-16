#!/usr/bin/env python3
"""Read-only installation check; no ROS nodes, motor ports or controllers.

Run after installation, as colcon test does. Integration recording is verified
separately in an isolated ROS domain, never by launching real.launch.py.
"""

import os
from pathlib import Path
import subprocess
import sys

import yaml


def main():
    prefix = Path(sys.argv[1])
    share = prefix / 'share/custom_dog_control'
    script = prefix / 'lib/custom_dog_control/record_latest_rosbag.py'
    qos = share / 'config/rosbag_qos.yaml'
    launch = share / 'launch/real.launch.py'
    for path in (script, qos, launch):
        if not path.is_file():
            raise RuntimeError(f'Missing installed rosbag asset: {path}; rebuild/install the package')
    if not os.access(script, os.X_OK):
        raise RuntimeError(f'Installed recorder is not executable: {script}')

    profiles = yaml.safe_load(qos.read_text())
    if not isinstance(profiles, dict):
        raise RuntimeError(f'Invalid installed rosbag QoS map: {qos}')
    for topic in ('/imu', '/joint_states', '/nmpc_wbc_controller/diagnostics'):
        if profiles.get(topic, {}).get('reliability') != 'best_effort':
            raise RuntimeError(f'Installed QoS does not accept BEST_EFFORT on {topic}')
    for topic in ('/robot_description', '/tf_static'):
        if profiles.get(topic, {}).get('durability') != 'transient_local':
            raise RuntimeError(f'Installed QoS cannot capture pre-existing latched {topic}')

    # --help exits before creating any ROS node, output directory or child.
    help_result = subprocess.run([str(script), '--help'], capture_output=True,
                                 text=True, check=True, timeout=10)
    for option in ('--output-dir', '--qos-profile-overrides-path'):
        if option not in help_result.stdout:
            raise RuntimeError(f'Installed recorder does not support launch argument {option}')
    print(f'Installed recorder and QoS validated: {prefix}')


if __name__ == '__main__':
    main()
