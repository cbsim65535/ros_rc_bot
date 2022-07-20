#!/usr/bin/env python

import rospy
import threading
import traceback
import time
import math

from pwm_msgs.msg import PwmCtrl
from geometry_msgs.msg import TwistStamped


class RcBot:

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

    REVERSE_ACCEL_SIZE_PER_ONE = (REVERSE_DRIVING_MIN_PULSE - ESC_MIN_PULSE) * 1.0

    PULSE_SPEED_ZERO = 2048

    def __init__(self):
        rospy.init_node("rc_car", anonymous=True)

        self.__speed = 0
        self.__zoom = -RcBot.ZOOM_MIN_PULSE

        rate = rospy.Rate(10)  # 10hz

        self.pub_pwm_set = rospy.Publisher("/pwm_ctrl/set", PwmCtrl, queue_size=10)

        rospy.Subscriber("/cmd_vel", TwistStamped, self.on_cmd_vel)
        # rospy.Subscriber("/camera/focus_ctrl", TwistStamped, self.on_focus_ctrl)

        sec = 0
        self.set_esc_pwm(RcBot.PULSE_SPEED_ZERO)

    def set_esc_pwm(self, pulse):
        pulse = int(pulse)
        if pulse < RcBot.PULSE_SPEED_ZERO:
            if RcBot.REVERSE_DRIVING_MIN_PULSE < pulse:
                pulse = RcBot.PULSE_SPEED_ZERO
        if pulse > RcBot.PULSE_SPEED_ZERO:
            if RcBot.DRIVING_MIN_PULSE > pulse:
                pulse = RcBot.PULSE_SPEED_ZERO
        if RcBot.ESC_MIN_PULSE > pulse:
            pulse = RcBot.PULSE_SPEED_ZERO
        if RcBot.ESC_MAX_PULSE < pulse:
            pulse = RcBot.PULSE_SPEED_ZERO

        self.pub_pwm_set.publish(PwmCtrl(self.CH_ESC, pulse))

    def on_focus_ctrl(self, msg):
        focus = msg.twist.angular.y * RcBot.FOCUS_SIZE_PER_ONE
        if focus > RcBot.MAX_FOCUS_PULSE_SIZE:
            focus = RcBot.MAX_FOCUS_PULSE_SIZE
        if focus < -RcBot.MAX_FOCUS_PULSE_SIZE:
            focus = -RcBot.MAX_FOCUS_PULSE_SIZE

        pulse = RcBot.FOCUS_CENTER_PULSE + focus
        self.set_focus_pwm(pulse)

        zoom = self.__zoom - (msg.twist.angular.x * RcBot.ZOOM_SIZE_PER_ONE)
        if zoom > RcBot.ZOOM_MAX_PULSE:
            zoom = RcBot.ZOOM_MAX_PULSE
        if zoom < RcBot.ZOOM_MIN_PULSE:
            zoom = RcBot.ZOOM_MIN_PULSE
        self.__zoom = zoom

        pulse = RcBot.ZOOM_CENTER_PULSE + zoom
        self.set_zoom_pwm(pulse)
        return

    def set_focus_pwm(self, pulse):
        pulse = int(pulse)
        self.pub_pwm_set.publish(PwmCtrl(self.CH_FOCUS, pulse))

    def set_zoom_pwm(self, pulse):
        pulse = int(pulse)
        self.pub_pwm_set.publish(PwmCtrl(self.CH_ZOOM, pulse))

    def set_steer_pwm(self, pulse):
        pulse = int(pulse)
        self.pub_pwm_set.publish(PwmCtrl(self.CH_SERVO, pulse))

    def on_cmd_vel(self, msg):
        # rospy.loginfo(msg)

        steer = msg.twist.angular.z * RcBot.STEER_SIZE_PER_ONE
        if steer > RcBot.MAX_STEER_PULSE_SIZE:
            steer = RcBot.MAX_STEER_PULSE_SIZE
        if steer < -RcBot.MAX_STEER_PULSE_SIZE:
            steer = -RcBot.MAX_STEER_PULSE_SIZE

        pulse = RcBot.STEER_CENTER_PULSE + steer
        self.set_steer_pwm(pulse)

        target_speed = 0.0
        accel = 0.0
        if msg.twist.linear.x > 0:
            target_speed = msg.twist.linear.x * RcBot.ACCEL_SIZE_PER_ONE
            accel = RcBot.ACCEL_SIZE_PER_ONE / RcBot.ACCEL_STEP
        elif msg.twist.linear.x < 0:
            target_speed = msg.twist.linear.x * RcBot.REVERSE_ACCEL_SIZE_PER_ONE
            accel = RcBot.REVERSE_ACCEL_SIZE_PER_ONE / RcBot.ACCEL_STEP
        else:
            self.__speed = 0.0

        if target_speed > self.__speed:
            self.__speed += accel
        if target_speed < self.__speed:
            self.__speed -= accel

        pulse = RcBot.PULSE_SPEED_ZERO
        if self.__speed < -RcBot.REVERSE_ACCEL_SIZE_PER_ONE:
            self.__speed = -RcBot.REVERSE_ACCEL_SIZE_PER_ONE
        if self.__speed > RcBot.ACCEL_SIZE_PER_ONE:
            self.__speed = RcBot.ACCEL_SIZE_PER_ONE
        if self.__speed < 0.0:
            pulse = RcBot.REVERSE_DRIVING_MIN_PULSE + self.__speed
        if self.__speed > 0.0:
            pulse = RcBot.DRIVING_MIN_PULSE + self.__speed

        self.set_esc_pwm(pulse)

    def stop(self):
        self.__is_loop = False


if __name__ == "__main__":
    try:
        rcbot = RcBot()
        rospy.spin()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    finally:
        traceback.print_exc()
        rcbot.stop()
