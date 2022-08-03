#!/usr/bin/env python

import rclpy
import threading
import traceback
import time

from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

from basecam_msgs.srv import (
    BasecamSetMotor,
    BasecamResetFollowOffset,
)


class TestCmdVel(Node):
    def __init__(self):
        super().__init__("test_cmd_vel_node")

        self.rate = self.create_rate(10)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        x = 0
        dx = 1
        z = 0
        dz = 1

        while rclpy.ok():
            twist_stapmed = TwistStamped()
            twist_stapmed.header.stamp = self.get_clock().now().to_msg()
            twist_stapmed.twist.linear.x = x
            twist_stapmed.twist.angular.z = z
            if x >= 0.5:
                dx = -1
            if x <= 0.5:
                dx = 1
            if z >= 0.5:
                dz = -1
            if z <= 0.5:
                dz = 1
            x += 0.1 * dx
            z += 0.1 * dz


def main(args=None):
    rclpy.init(args=args)
    manual = TestCmdVel()
    rclpy.spin(manual)
    manual.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
