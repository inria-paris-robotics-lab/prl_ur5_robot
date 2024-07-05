#!/usr/bin/env python
"""Script to initialize the planning scene."""

import os
import sys

import geometry_msgs
import moveit_commander
import rospkg
import rospy


def run():
    """Initialize the planning scene.
    """
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    # Remove all objects from the planning scene
    scene.remove_world_object()

    # Add the table model to the planning scene
    r = rospkg.RosPack()
    filename = os.path.join(
        r.get_path('prl_ur5_description'), 'model', 'vention_table.stl')

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'prl_ur5_base'
    pose.pose.orientation.w = 1.0

    scene.add_mesh('table', pose, filename)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('init_planning_scene')
    run()
