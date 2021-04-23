#!/usr/bin/env python
"""Wrench message retranslator node for the Robotiq FT300 sensor."""

import threading
import time
import rospy
import socket
from geometry_msgs.msg import WrenchStamped, Vector3


def run():
    topic = rospy.get_param('~topic', default='ft_wrench')
    pub = rospy.Publisher(topic, WrenchStamped,
                          tcp_nodelay=True, queue_size=100)

    frame_id = rospy.get_param('~frame_id', 'robotiq_ft_frame_id')

    msg = WrenchStamped()
    msg.header.frame_id = frame_id

    host = rospy.get_param('~ip_address')
    port = rospy.get_param('~port', 63351)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((host, port))
    except Exception as e:
        rospy.logerr(
            'Cannot connect to the FT sensor server ({}:{}): {}'.format(host, port, e))
        return

    try:
        buffer = b''
        while not rospy.is_shutdown():
            buffer += sock.recv(1024)

            ind_close = buffer.rfind(b')')
            ind_open = buffer.rfind(b'(', 0, ind_close)
            if ind_close - ind_open > 0:
                package = buffer[ind_open + 1:ind_close]
                buffer = buffer[ind_close+1:]
                fx, fy, fz, tx, ty, tz = map(float, package.split(b' , '))

                msg.header.stamp = rospy.Time.now()
                msg.wrench.force = Vector3(fx, fy, fz)
                msg.wrench.torque = Vector3(tx, ty, tz)
                pub.publish(msg)
    finally:
        sock.close()


if __name__ == '__main__':
    rospy.init_node('ft_sensor_node', anonymous=True)
    run()
