#!/usr/bin/python
"""Demonstration of arm moving using MoveIt! interface."""

import sys
from math import pi

import moveit_commander
import rospy

from utils import make_pose


def run():
    """Move the arm in joint space then in Cartesian space.
    """
    robot = moveit_commander.RobotCommander()

    start_pose = robot.left_arm.get_current_pose()
    target_pose = make_pose((-0.1, 0.1, 0.4), (pi/2, 0, pi))

    # move in joint space with obstacle avoidance
    robot.left_arm.set_pose_target(target_pose)
    success = robot.left_arm.go(wait=True)
    if not success:
        return

    # move back in straight line in cartesian space
    path, fraction = robot.left_arm.compute_cartesian_path(
        waypoints=[start_pose.pose], eef_step=0.01, jump_threshold=0.0)
    if fraction < 1.0:
        return
    robot.left_arm.execute(path, wait=True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
