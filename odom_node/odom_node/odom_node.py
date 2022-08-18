#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading
import traceback

from ls7366r import LS7366R

import math
from math import sin, cos, pi

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

DISTANCE_WHEELS = 0.202
LEFT_WHEEL_DIAMETER = 0.127
RIGHT_WHEEL_DIAMETER = 0.127
TOTAL_CODE = 2880


class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__("odom_node")
        self._is_loop = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.left_encoder = LS7366R(0, 1000000, 4)
        self.right_encoder = LS7366R(1, 1000000, 4)

        self._count_left = 0
        self._count_right = 0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.odom_tf_broadcaster = TransformBroadcaster(self)

        self.read_encoder = threading.Thread(target=self.read_encoder, args=()).start()
        self.loop = threading.Thread(target=self._loop, args=()).start()

    def read_encoder(self):
        RATE = 3000
        r = self.create_rate(RATE)
        while True:
            self._prev_left = self._count_left
            self._prev_right = self._count_right

            self._count_left = self.left_encoder.readCounter()
            self._count_right = -self.right_encoder.readCounter()

            r.sleep()

    def _loop(self):
        RATE = 10
        r = self.create_rate(RATE)
        while self._is_loop:
            current_time = self.get_clock().now()

            velocity_left = (
                -(self._count_left - self._prev_left)
                / float(TOTAL_CODE)
                * float(RATE)
                * math.pi
                * LEFT_WHEEL_DIAMETER
            )
            velocity_right = (
                -(self._count_right - self._prev_right)
                / float(TOTAL_CODE)
                * float(RATE)
                * math.pi
                * RIGHT_WHEEL_DIAMETER
            )

            old_left = self._count_left
            old_right = self._count_right

            count_left = self._count_left - self._prev_left
            count_right = self._count_right - self._prev_right

            velocity_linear = (velocity_right + velocity_left) / 2.0
            velocity_angular = (velocity_right - velocity_left) / DISTANCE_WHEELS

            vx = velocity_linear
            vy = 0.0
            vth = velocity_angular

            dt = (current_time - self.last_time).nanoseconds / 1000000
            delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
            delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
            delta_th = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            odom_quat = quaternion_from_euler(0, 0, self.th)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = current_time.to_msg()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = vth

            self.odom_pub.publish(odom)

            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = odom_quat[0]
            t.transform.rotation.y = odom_quat[1]
            t.transform.rotation.z = odom_quat[2]
            t.transform.rotation.w = odom_quat[3]

            self.odom_tf_broadcaster.sendTransform(t)

            self.last_time = current_time

            r.sleep()

    def stop(self):
        self._is_loop = False


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def main(args=None):
    rclpy.init(args=args)
    odom = OdometryPublisherNode()
    rclpy.spin(odom)
    odom.stop()
    odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
