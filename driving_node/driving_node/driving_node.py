#!/usr/bin/env python

import rclpy
import threading
import traceback

from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped


class DrivingNode(Node):

    CH_SERVO = 0
    CH_ESC = 1
    CH_FOCUS = 2
    CH_ZOOM = 3

    ZOOM_SIZE_PER_ONE = 10
    ZOOM_CENTER_PULSE = 369
    ZOOM_MIN_PULSE = -60
    ZOOM_MAX_PULSE = 60

    FOCUS_SIZE_PER_ONE = 50
    FOCUS_CENTER_PULSE = 369
    MAX_FOCUS_PULSE_SIZE = 50

    STEER_SIZE_PER_ONE = 100
    STEER_CENTER_PULSE = 369
    MAX_STEER_PULSE_SIZE = 100

    ESC_MAX_PULSE = 2252
    ESC_MIN_PULSE = 1638

    DRIVING_MIN_PULSE = 2068  # 2048
    REVERSE_DRIVING_MIN_PULSE = 1843

    ACCEL_SIZE_PER_ONE = (ESC_MAX_PULSE - DRIVING_MIN_PULSE) * 1.0
    ACCEL_STEP = 20.0

    REVERSE_ACCEL_SIZE_PER_ONE = (
        REVERSE_DRIVING_MIN_PULSE - ESC_MIN_PULSE) * 1.0

    PULSE_SPEED_ZERO = 2048

    def __init__(self):
        super().__init__("driving_node")

        self.__speed = 0
        self.__zoom = -DrivingNode.ZOOM_MIN_PULSE

        self.rate = self.create_rate(10)

        self.pub_pwm_set_0 = self.create_publisher(
            Int32, "/pwm_ctrl/set/a", 10)
        self.pub_pwm_set_1 = self.create_publisher(
            Int32, "/pwm_ctrl/set/b", 10)
        self.pub_pwm_set_2 = self.create_publisher(
            Int32, "/pwm_ctrl/set/c", 10)
        self.pub_pwm_set_3 = self.create_publisher(
            Int32, "/pwm_ctrl/set/d", 10)

        self.sub_cmd_vel = self.create_subscription(
            TwistStamped, "/cmd_vel", self.on_cmd_vel, 10
        )
        self.sub_cmd_vel  # prevent unused variable warning
        self.set_esc_pwm(DrivingNode.PULSE_SPEED_ZERO)

    def set_esc_pwm(self, pulse):
        pulse = int(pulse)
        if pulse < DrivingNode.PULSE_SPEED_ZERO:
            if DrivingNode.REVERSE_DRIVING_MIN_PULSE < pulse:
                pulse = DrivingNode.PULSE_SPEED_ZERO
        if pulse > DrivingNode.PULSE_SPEED_ZERO:
            if DrivingNode.DRIVING_MIN_PULSE > pulse:
                pulse = DrivingNode.PULSE_SPEED_ZERO
        if DrivingNode.ESC_MIN_PULSE > pulse:
            pulse = DrivingNode.PULSE_SPEED_ZERO
        if DrivingNode.ESC_MAX_PULSE < pulse:
            pulse = DrivingNode.PULSE_SPEED_ZERO

        msg = Int32()
        msg.data = pulse
        self.pub_pwm_set_1.publish(msg)

    def on_focus_ctrl(self, msg):
        focus = msg.twist.angular.y * DrivingNode.FOCUS_SIZE_PER_ONE
        if focus > DrivingNode.MAX_FOCUS_PULSE_SIZE:
            focus = DrivingNode.MAX_FOCUS_PULSE_SIZE
        if focus < -DrivingNode.MAX_FOCUS_PULSE_SIZE:
            focus = -DrivingNode.MAX_FOCUS_PULSE_SIZE

        pulse = DrivingNode.FOCUS_CENTER_PULSE + focus
        self.set_focus_pwm(pulse)

        zoom = self.__zoom - (msg.twist.angular.x *
                              DrivingNode.ZOOM_SIZE_PER_ONE)
        if zoom > DrivingNode.ZOOM_MAX_PULSE:
            zoom = DrivingNode.ZOOM_MAX_PULSE
        if zoom < DrivingNode.ZOOM_MIN_PULSE:
            zoom = DrivingNode.ZOOM_MIN_PULSE
        self.__zoom = zoom

        pulse = DrivingNode.ZOOM_CENTER_PULSE + zoom
        self.set_zoom_pwm(pulse)
        return

    def set_focus_pwm(self, pulse):
        pulse = int(pulse)
        msg = Int32()
        msg.data = pulse
        self.pub_pwm_set_2.publish(msg)

    def set_zoom_pwm(self, pulse):
        pulse = int(pulse)
        msg = Int32()
        msg.data = pulse
        self.pub_pwm_set_3.publish(msg)

    def set_steer_pwm(self, pulse):
        pulse = int(pulse)
        msg = Int32()
        msg.data = pulse
        self.pub_pwm_set_0.publish(msg)

    def on_cmd_vel(self, msg):
        # rospy.loginfo(msg)

        steer = msg.twist.angular.z * DrivingNode.STEER_SIZE_PER_ONE
        if steer > DrivingNode.MAX_STEER_PULSE_SIZE:
            steer = DrivingNode.MAX_STEER_PULSE_SIZE
        if steer < -DrivingNode.MAX_STEER_PULSE_SIZE:
            steer = -DrivingNode.MAX_STEER_PULSE_SIZE

        pulse = DrivingNode.STEER_CENTER_PULSE + steer
        self.set_steer_pwm(pulse)

        target_speed = 0.0
        accel = 0.0
        if msg.twist.linear.x > 0:
            target_speed = msg.twist.linear.x * DrivingNode.ACCEL_SIZE_PER_ONE
            accel = DrivingNode.ACCEL_SIZE_PER_ONE / DrivingNode.ACCEL_STEP
        elif msg.twist.linear.x < 0:
            target_speed = msg.twist.linear.x * DrivingNode.REVERSE_ACCEL_SIZE_PER_ONE
            accel = DrivingNode.REVERSE_ACCEL_SIZE_PER_ONE / DrivingNode.ACCEL_STEP
        else:
            self.__speed = 0.0

        if target_speed > self.__speed:
            self.__speed += accel
        if target_speed < self.__speed:
            self.__speed -= accel

        pulse = DrivingNode.PULSE_SPEED_ZERO
        if self.__speed < -DrivingNode.REVERSE_ACCEL_SIZE_PER_ONE:
            self.__speed = -DrivingNode.REVERSE_ACCEL_SIZE_PER_ONE
        if self.__speed > DrivingNode.ACCEL_SIZE_PER_ONE:
            self.__speed = DrivingNode.ACCEL_SIZE_PER_ONE
        if self.__speed < 0.0:
            pulse = DrivingNode.REVERSE_DRIVING_MIN_PULSE + self.__speed
        if self.__speed > 0.0:
            pulse = DrivingNode.DRIVING_MIN_PULSE + self.__speed

        self.set_esc_pwm(pulse)


def main(args=None):
    rclpy.init(args=args)
    driving = DrivingNode()
    rclpy.spin(driving)
    driving.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
