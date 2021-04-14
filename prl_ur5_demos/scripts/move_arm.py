#!/usr/bin/env python
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

    # Configure the planning pipeline
    robot.both_arms.set_max_velocity_scaling_factor(0.5)
    robot.both_arms.set_max_acceleration_scaling_factor(0.2)

    # Move in joint space with obstacle avoidance
    robot.left_arm.set_pose_target(target_pose)
    success = robot.left_arm.go(wait=True)
    if not success:
        return

    # Move back in straight line in cartesian space
    path, fraction = robot.left_arm.compute_cartesian_path(
        waypoints=[start_pose.pose], eef_step=0.01, jump_threshold=0.0)
    if fraction < 1.0:
        return
    # 'compute_cartesian_path' does not use 'velocity_scaling_factor'
    # and 'acceleration_scaling_factor', so we have to replan manually
    state = robot.get_current_state()
    path = robot.left_arm.retime_trajectory(
        state, path, 0.5, 0.5, 'time_optimal_trajectory_generation')
    robot.left_arm.execute(path, wait=True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
