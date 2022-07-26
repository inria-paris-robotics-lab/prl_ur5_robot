#!/usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import (GripperCommandAction, GripperCommandFeedback, GripperCommandResult)
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF


class FakeGripperControlServer:
    """A fake gripper controller class that implements all the propper interfaces and execute every command instantly"""

    def __init__(self):
        """Constructor for a class FakeGripperControlServer."""
        self._state_publish_rate = rospy.get_param("~state_publish_rate", 50)
        self._action_monitor_rate = rospy.get_param('~action_monitor_rate', 5)
        self._joint_name = rospy.get_param('~joint', 'gripper_joint')
        self._mimics = _get_joint_info(self._joint_name)
        self.pos = 0 # The current 'position' of the gripper

        self._server = actionlib.SimpleActionServer(
            '~gripper_cmd', GripperCommandAction, execute_cb=self.execute,
            auto_start=False)

        self._joint_states_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)

    def start(self):
        """Start the server."""
        self._server.start()

    def execute(self, goal):
        """Action server callback.
        """
        self.pos = goal.command.position
        if self._server.is_active():
            result = GripperCommandResult(
                position=self.pos,
                effort=0,
                stalled=False,
                reached_goal=True,
            )
            self._server.set_succeeded(result)
            return

    def publish_state_cb(self, timer):
        """Periodic task to publish the joint states message.
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name.append(self._joint_name)
        msg.position.append(self.pos)
        msg.velocity.append(0.0)
        msg.effort.append(0.0)

        for name, _ in self._mimics.items():
            msg.name.append(name)
            msg.position.append(self.pos)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self._joint_states_pub.publish(msg)


def _get_joint_info(joint_name):
    mimics = {}

    robot = URDF.from_parameter_server(key='/robot_description')
    for joint in robot.joints:
        if joint.name == joint_name:
            pass
        elif joint.mimic is not None:
            if joint.mimic.joint == joint_name:
                mimics[joint.name] = joint.mimic

    return mimics


def main():
    rospy.init_node('fake_gripper', anonymous=True)
    srv = FakeGripperControlServer()
    srv.start()
    rospy.spin()


if __name__ == '__main__':
    main()