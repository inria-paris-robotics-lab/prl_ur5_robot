#!/usr/bin/env python
"""Demonstration of two arm simultaneous moving using MoveIt! interface."""

import sys
from copy import deepcopy
from math import pi

import moveit_commander
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

from utils import make_pose, shift_pose


def run():
    """Move both arms to a start position in joint space, then move both arms
    together in Cartesian space.

    Note: There is no MoveIt interface for planning the path for many arms in
          Cartesian space, as a workaround we plan separately and then merge
          the paths.
    """
    robot = moveit_commander.RobotCommander()

    # Configure the planning pipeline
    robot.both_arms.set_planner_id('RRTstar')
    robot.both_arms.set_planning_time(10.0)
    robot.both_arms.set_max_velocity_scaling_factor(0.5)
    robot.both_arms.set_max_acceleration_scaling_factor(0.5)

    # Define waypoints in Cartesian space
    waypoints = [
        make_pose((0.0, 0.1, 0.4), (pi/2, 0, pi)),
        make_pose((0.1, 0.1, 0.4), (pi/2, 0, pi)),
        make_pose((0.1, 0.1, 0.5), (pi/2, 0, pi)),
        make_pose((0.0, 0.1, 0.5), (pi/2, 0, pi)),
        make_pose((0.0, 0.1, 0.4), (pi/2, 0, pi)),
    ]
    left_waypoints = [shift_pose(wp, dx=-0.3) for wp in waypoints]
    right_waypoints = [shift_pose(wp, dx=0.3) for wp in waypoints]

    # Move both arms simultaneously in joint space with obstacle avoidance
    robot.both_arms.set_pose_target(
        left_waypoints[0], end_effector_link="left_tool")
    robot.both_arms.set_pose_target(
        right_waypoints[0], end_effector_link="right_tool")

    success = robot.both_arms.go(wait=True)
    if not success:
        return

    # Move in cartesian space through the waypoints
    left_path, left_fraction = robot.left_arm.compute_cartesian_path(
        waypoints=left_waypoints[1:], eef_step=0.01, jump_threshold=1.5)

    right_path, right_fraction = robot.right_arm.compute_cartesian_path(
        waypoints=right_waypoints[1:], eef_step=0.01, jump_threshold=1.5)

    if left_fraction < 1.0 or right_fraction < 1.0:
        rospy.logerr('Failed to plan cartesian path')
        return

    path = _merge_path(robot, left_path, right_path)
    robot.left_arm.execute(path, wait=True)

    # move arms to the default position in joint space
    robot.both_arms.set_named_target('default')
    robot.both_arms.go(wait=True)


def _merge_path(robot, left_path, right_path):

    def sec(point):
        return point.time_from_start.to_sec()

    # equalize the duration
    left_sec = sec(left_path.joint_trajectory.points[-1])
    right_sec = sec(right_path.joint_trajectory.points[-1])

    state_in = robot.get_current_state()
    algorithm = 'time_optimal_trajectory_generation'

    factor = min(1.0, right_sec / left_sec)
    left_path = robot.left_arm.retime_trajectory(
        state_in, left_path, factor, 0.5, algorithm)

    factor = min(1.0, left_sec / right_sec)
    right_path = robot.right_arm.retime_trajectory(
        state_in, right_path, factor, 0.5, algorithm)

    # merge trajectories
    dest = left_path.joint_trajectory
    src = right_path.joint_trajectory

    dest.joint_names += src.joint_names

    i = 0
    for point in dest.points:
        if (i < len(src.points) - 1 and
                abs(sec(point)-sec(src.points[i+1])) < abs(sec(point)-sec(src.points[i]))):
            i += 1
        point.positions += src.points[i].positions
        point.velocities += src.points[i].velocities
        point.accelerations += src.points[i].accelerations

    return left_path


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
