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

        self.rate = self.create_rate(5)
        self.pub_camera_ctrl = self.create_publisher(
            TwistStamped, "/basecam/direct_ctrl", 10
        )
        self.y = 0.0
        self.dy = 1.0
        self.z = 0.0
        self.dz = 1.0

        self.is_loop = True

        threading.Thread(
            target=self.loop,
            args=(),
        ).start()

    def stop(self):
        self.is_loop = False

    def loop(self):
        while self.is_loop:
            if self.y >= 0.5:
                self.dy = -1
            if self.y <= -0.5:
                self.dy = 1
            if self.z >= 0.5:
                self.dz = -1
            if self.z <= -0.5:
                self.dz = 1
            self.y += 0.02 * self.dy
            self.z += 0.02 * self.dz
            print("%f,%f" % (self.y, self.z))
            twist_stapmed = TwistStamped()
            twist_stapmed.header.stamp = self.get_clock().now().to_msg()
            twist_stapmed.twist.angular.y = self.y
            twist_stapmed.twist.angular.z = self.z
            self.pub_camera_ctrl.publish(twist_stapmed)
            self.rate.sleep()


def main(args=None):
    print("init")
    rclpy.init(args=args)
    manual = TestCamera()
    rclpy.spin(manual)
    manual.stop()
    manual.destroy_node()
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    main()
