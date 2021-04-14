#!/usr/bin/env python
"""Demonstration of an introspection using MoveIt! interface."""

from __future__ import print_function

import sys
from math import pi

import moveit_commander
import rospy

from utils import make_pose


def run():
    """Print moving groups info.
    """
    robot = moveit_commander.RobotCommander()
    groups = {name: robot.get_group(name) for name in robot.get_group_names()}

    print('Available groups:')
    for name, group in groups.items():
        print(' {}:'.format(name))
        print('  end effector: {}'.format(group.get_end_effector_link()))
        print('  joints:')
        for joint in group.get_joints():
            print('  - {}'.format(joint))
        print('')

    print('Default planning frame: {}'.format(robot.get_planning_frame()))
    print('')

    print('Current state:')
    for joint, value in robot.get_current_variable_values().items():
        print('- {}: {:.2f}'.format(joint, value))
    print('')


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
