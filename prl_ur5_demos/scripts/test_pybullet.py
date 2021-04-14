#!/usr/bin/env python3
"""Test the robot model in PyBullet."""

import re
import subprocess
import tempfile
import time
import xml
import contextlib

import pybullet as pb
import rospkg
from pybullet_utils.bullet_client import BulletClient


def run():
    """Load URDF in PyBullet, move arms from initial to target state.
    """
    # Start simulation and load URDF
    client = BulletClient(pb.GUI)
    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    with prl_ur5_robot_urdf() as filename:
        body_id = client.loadURDF(filename, useFixedBase=1)

    client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    # Map joint names to indices
    joint_map = {}
    for i in range(client.getNumJoints(body_id)):
        info = client.getJointInfo(body_id, i)
        name = info[1].decode('utf-8', 'strict')
        joint_map[name] = i

    # Show joints map
    print('Robot joints:')
    for name, i in joint_map.items():
        print('{}: {}'.format(i, name))

    # Reset joints at initial state
    init_state = {
        'left_elbow_joint': -1.57,
        'left_shoulder_lift_joint': -1.57,
        'left_shoulder_pan_joint': -1.57,
        'left_wrist_1_joint': 0,
        'left_wrist_2_joint': 1.57,
        'left_wrist_3_joint': -0.785,
        'right_elbow_joint': 1.57,
        'right_shoulder_lift_joint': -1.57,
        'right_shoulder_pan_joint': 1.57,
        'right_wrist_1_joint': -3.14,
        'right_wrist_2_joint': -1.57,
        'right_wrist_3_joint': 0.785,
    }
    for name, value in init_state.items():
        client.resetJointState(body_id, joint_map[name], value)

    # Apply target state
    target_state = {
        'left_elbow_joint': -1.57,
        'left_shoulder_lift_joint': -1.57,
        'left_shoulder_pan_joint': -1.57,
        'left_wrist_1_joint': -1.57,
        'left_wrist_2_joint': 2.36,
        'left_wrist_3_joint': -1.57,
        'right_elbow_joint': 1.57,
        'right_shoulder_lift_joint': -1.57,
        'right_shoulder_pan_joint': 1.57,
        'right_wrist_1_joint': -1.57,
        'right_wrist_2_joint': -2.36,
        'right_wrist_3_joint': 1.57,
    }
    client.setJointMotorControlArray(
        bodyUniqueId=body_id,
        jointIndices=[joint_map[name] for name in target_state],
        controlMode=pb.POSITION_CONTROL,
        targetPositions=[value for value in target_state.values()],
        forces=[10]*len(target_state))

    # Simulate until the user to close the window
    while client.isConnected():
        client.stepSimulation()
        time.sleep(0.1)


@contextlib.contextmanager
def prl_ur5_robot_urdf():
    """Generate and adapt URDF description file.
    """
    # Generate URDF from xacro files
    proc = subprocess.run(
        'xacro `rospack find prl_ur5_description`/urdf/prl_ur5_robot.urdf.xacro',
        shell=True, capture_output=True)

    urdf = xml.dom.minidom.parseString(proc.stdout.decode('utf-8', 'strict'))

    # Fix links without inertial parameters
    for link in urdf.getElementsByTagName('link'):
        if link.getElementsByTagName('inertial'):
            continue
        inertial = urdf.createElement('inertial')

        mass = urdf.createElement('mass')
        mass.setAttribute('value', '0.001')
        inertial.appendChild(mass)

        inertia = urdf.createElement('inertia')
        inertia.setAttribute('ixx', '0.00001')
        inertia.setAttribute('iyy', '0.00001')
        inertia.setAttribute('izz', '0.00001')
        inertia.setAttribute('ixy', '0')
        inertia.setAttribute('ixz', '0')
        inertia.setAttribute('iyz', '0')
        inertial.appendChild(inertia)

        link.appendChild(inertial)

    # Replace package paths with absolute ones
    packages = {}
    r = rospkg.RosPack()

    def sub_path(match):
        pkg_name = match.group(1)
        pkg_path = r.get_path(pkg_name)
        packages[pkg_name] = pkg_path
        return pkg_path

    for mesh in urdf.getElementsByTagName('mesh'):
        node = mesh.getAttributeNode('filename')
        node.value = re.sub(r'package://([a-zA-Z_0-9]*)', sub_path, node.value)

    # Show dependencies
    print('The URDF description uses resources from packages:')
    for name, path in packages.items():
        print('{}: {}'.format(name, path))

    # Dump URDF to temporary file
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        urdf.writexml(f, encoding='utf-8')
        f.flush()
        yield f.name


if __name__ == '__main__':
    run()
