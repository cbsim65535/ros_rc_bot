#!/usr/bin/env python

import rospy
import threading
import traceback
import time

from ls7366r import LS7366R

import math
from math import sin, cos, pi
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

DISTANCE_WHEELS = 0.202
LEFT_WHEEL_DIAMETER = 0.127
RIGHT_WHEEL_DIAMETER = 0.127
TOTAL_CODE = 2880


class OdometryPublisher:
    def __init__(self):
        rospy.init_node("odometry_publisher", anonymous=True)
        self._is_loop = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        self.left_encoder = LS7366R(0, 1000000, 4)
        self.right_encoder = LS7366R(1, 1000000, 4)

        self._count_left = 0
        self._count_right = 0

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.read_encoder = threading.Thread(target=self.read_encoder, args=()).start()
        self.loop = threading.Thread(target=self._loop, args=()).start()

    def read_encoder(self):
        r = rospy.Rate(3000)
        while True:
            self._prev_left = self._count_left
            self._prev_right = self._count_right

            self._count_left = -self.left_encoder.readCounter()
            self._count_right = self.right_encoder.readCounter()

            rospy.loginfo("A %d, %d" % (self._count_left, self._count_right))
            r.sleep()

    def _loop(self):
        RATE = 10
        r = rospy.Rate(RATE)
        while self._is_loop:
            current_time = rospy.Time.now()

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
            vy = 0
            vth = velocity_angular

            dt = (current_time - self.last_time).to_sec()
            delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
            delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
            delta_th = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # rospy.loginfo(
            #     "(x=%f, y=%f, th=%f, Dx=%f, Dy=%f, Dt=%f, Cl=%d, Cr=%d)"
            #     % (
            #         self.x,
            #         self.y,
            #         (self.th % math.pi),
            #         delta_x,
            #         delta_y,
            #         dt,
            #         count_left,
            #         count_right,
            #     )
            # )

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = current_time
            odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # rospy.loginfo(odom)

            self.odom_pub.publish(odom)

            self.last_time = current_time

            r.sleep()

    def stop(self):
        self._is_loop = False


if __name__ == "__main__":
    try:
        odom = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    finally:
        traceback.print_exc()
        odom.stop()
