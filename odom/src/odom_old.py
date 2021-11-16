#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
import threading
import traceback
import time

import math
from math import sin, cos, pi
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

DISTANCE_WHEELS = 0.202
LEFT_WHEEL_DIAMETER = 0.127
RIGHT_WHEEL_DIAMETER = 0.127
TOTAL_CODE = 15


class Encoder:
    def __init__(self, a_pin, b_pin):
        self.a_pin = a_pin
        self.b_pin = b_pin
        GPIO.setup(self.a_pin, GPIO.IN)
        GPIO.setup(self.b_pin, GPIO.IN)

        self.last_delta = 0
        self.r_seq = self.rotation_sequence()

        # steps_per_cycle and remainder are only used in get_cycles which
        # returns a coarse-granularity step count.  By default
        # steps_per_cycle is 4 as there are 4 steps per
        # detent on my encoder, and get_cycles() will return -1 or 1
        # for each full detent step.
        self.steps_per_cycle = 4
        self.remainder = 0

    # Returns the quadrature encoder state converted into
    # a numerical sequence 0,1,2,3,0,1,2,3...
    #
    # Turning the encoder clockwise generates these
    # values for switches B and A:
    #  B A B^A B^A|B<<1
    #  0 0  0     0
    #  0 1  1     1
    #  1 1  0     2
    #  1 0  1     3
    def rotation_sequence(self):
        a_state = GPIO.input(self.a_pin)
        b_state = GPIO.input(self.b_pin)
        r_seq = (a_state ^ b_state) | b_state << 1
        # print(a_state, b_state)
        return r_seq

    # Returns offset values of -2,-1,0,1,2
    def get_delta(self):
        delta = 0
        r_seq = self.rotation_sequence()
        if r_seq != self.r_seq:
            delta = (r_seq - self.r_seq) % 4
            if delta == 3:
                delta = -1
            elif delta == 2:
                delta = int(
                    math.copysign(delta, self.last_delta)
                )  # same direction as previous, 2 steps

            self.last_delta = delta
            self.r_seq = r_seq

        return delta

    def get_cycles(self):
        # python negative integers do not behave like they do in C.
        #   -1 // 2 = -1 (not 0)
        #   -1 % 2 =  1 (not -1)
        # // is integer division operator.  Note the behaviour of the / operator
        # when used on integers changed between python 2 and 3.
        # See http://www.python.org/dev/peps/pep-0238/
        self.remainder += self.get_delta()
        cycles = self.remainder // self.steps_per_cycle
        self.remainder %= self.steps_per_cycle  # remainder always remains positive
        return cycles


class OdometryPublisher:
    def __init__(self):
        rospy.init_node("odometry_publisher", anonymous=True)
        self._is_loop = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        GPIO.setmode(GPIO.BOARD)
        self.left_encoder = encoder = Encoder(13, 15)
        self.right_encoder = encoder = Encoder(16, 18)
        self._count_left = 0
        self._count_right = 0

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.read_encoder = threading.Thread(target=self.read_encoder, args=()).start()
        self.loop = threading.Thread(target=self._loop, args=()).start()

    def read_encoder(self):
        r = rospy.Rate(3000)
        while True:
            tmp = self.left_encoder.get_cycles()
            self._count_left += tmp
            tmp = self.right_encoder.get_cycles()
            self._count_right += tmp

            # rospy.loginfo("A %d, %d"%(self._count_left, self._count_right))
            r.sleep()

    def _loop(self):
        RATE = 10
        r = rospy.Rate(RATE)
        while self._is_loop:
            current_time = rospy.Time.now()

            velocity_left = (
                -self._count_left
                / float(TOTAL_CODE)
                * float(RATE)
                * math.pi
                * LEFT_WHEEL_DIAMETER
            )
            velocity_right = (
                -self._count_right
                / float(TOTAL_CODE)
                * float(RATE)
                * math.pi
                * RIGHT_WHEEL_DIAMETER
            )

            old_left = self._count_left
            old_right = self._count_right

            count_left = self._count_left
            count_right = self._count_right
            self._count_left = 0
            self._count_right = 0

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

            rospy.loginfo(
                "(x=%f, y=%f, th=%f, Dx=%f, Dy=%f, Dt=%f, Cl=%d, Cr=%d)"
                % (
                    self.x,
                    self.y,
                    (self.th % math.pi),
                    delta_x,
                    delta_y,
                    dt,
                    count_left,
                    count_right,
                )
            )

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
