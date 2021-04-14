#!/usr/bin/env python
"""Demonstration of gripper opening/closing using MoveIt! interface."""

import sys

import moveit_commander
import rospy


def run():
    """Check left and right grippers.
    """
    robot = moveit_commander.RobotCommander()

    # close / open left gripper
    robot.left_gripper.set_named_target('close')
    robot.left_gripper.go(wait=True)

    robot.left_gripper.set_named_target('open')
    robot.left_gripper.go(wait=True)

    # close / open right gripper
    robot.right_gripper.set_named_target('close')
    robot.right_gripper.go(wait=True)

    robot.right_gripper.set_named_target('open')
    robot.right_gripper.go(wait=True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
