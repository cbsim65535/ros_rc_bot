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


class TestCamera(Node):
    def __init__(self):
        super().__init__("test_camera_node")

        self.rate = self.create_rate(10)
        self.pub_camera_ctrl = self.create_publisher(
            TwistStamped, "/basecam/direct_ctrl", 10
        )
        y = 0.0
        dy = 1.0
        z = 0.0
        dz = 1.0

        while rclpy.ok():
            twist_stapmed = TwistStamped()
            twist_stapmed.header.stamp = self.get_clock().now().to_msg()
            twist_stapmed.twist.angular.y = y
            twist_stapmed.twist.angular.z = z
            if y >= 0.5:
                dy = -1
            if y <= 0.5:
                dy = 1
            if z >= 0.5:
                dz = -1
            if z <= 0.5:
                dz = 1
            y += 0.1 * dy
            z += 0.1 * dz
            print("%f,%f" % (y, z))
            self.pub_camera_ctrl.publish(twist_stapmed)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    manual = TestCamera()
    rclpy.spin(manual)
    manual.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
