#!/usr/bin/env python
"""Demonstration of the pick and place operation using MoveIt! interface."""

import sys
from math import pi

import moveit_commander
import rospy
from prl_ur5_demos.utils import (full_path, gazebo_delete_model,
                                 gazebo_spawn_urdf_model, make_grasp,
                                 make_location, make_pose, shift_pose)


def run():
    """Add an object model to the planning scene then pick up the object and place
    in different place.
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    init_pose = make_pose((0, 0, 0), (0, 0, 0), frame_id='prl_ur5_base')
    place_pose = make_pose((0, 0.1, 0), (0, 0, 0), frame_id='prl_ur5_base')

    # Spawn the coke can in Gazebo if it's a simulation
    if rospy.has_param('gazebo'):
        filename = full_path('models/coke_can.urdf')
        gazebo_spawn_urdf_model('coke_can', filename, init_pose)

    # Add the collision model to the planning scene
    filename = full_path('models/coke_can.stl')
    scene.add_mesh('coke_can', init_pose, filename)

    try:
        # Configure the planning pipeline
        robot.left_arm.set_planner_id('RRTstar')
        robot.left_arm.set_planning_time(10.0)
        robot.left_arm.set_num_planning_attempts(20)
        robot.left_arm.set_max_velocity_scaling_factor(0.5)
        robot.left_arm.set_max_acceleration_scaling_factor(0.5)

        # Allow collision between the table and the can during the pick and place
        robot.left_arm.set_support_surface_name('table')

        # Pick up the coke can
        eef_grasp_pose = make_pose(
            (-0.23, 0, 0.06), (pi/2, 0, pi/2), 'prl_ur5_base')
        grasp = make_grasp(robot.left_arm, robot.left_gripper, eef_grasp_pose)
        retcode = robot.left_arm.pick('coke_can', grasp, plan_only=False)
        if retcode != 1:
            return

        # Place the coke can
        location = make_location(
            robot.left_arm, robot.left_gripper, place_pose)
        retcode = robot.left_arm.place('coke_can', location, plan_only=False)
        if retcode != 1:
            return

    finally:
        # Remove the can from the planning scene
        scene.remove_attached_object(robot.left_arm.get_end_effector_link())
        scene.remove_world_object('coke_can')

        # Remove the can from the Gazebo scene if it's a simulation
        if rospy.has_param('gazebo'):
            gazebo_delete_model('coke_can')

        # Move arm to the default position in joint space
        robot.left_arm.set_named_target('default')
        robot.left_arm.go(wait=True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_example')
    run()
