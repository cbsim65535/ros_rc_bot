#!/usr/bin/env python

import rospy
import threading
import traceback
import time
import math

# Import the PCA9685 module.
import Adafruit_PCA9685

from std_msgs.msg import String
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

    ESC_MAX_PULSE = 3686
    ESC_MIN_PULSE = 409

    DRIVING_MIN_PULSE = 1228
    REVERSE_DRIVING_MIN_PULSE = 1228

    ACCEL_SIZE_PER_ONE = (ESC_MAX_PULSE - DRIVING_MIN_PULSE) * 0.2
    REVERSE_ACCEL_SIZE_PER_ONE = (REVERSE_DRIVING_MIN_PULSE - ESC_MIN_PULSE) * 0.4

    PULSE_SPEED_ZERO = 1228

    def __init__(self):
        rospy.init_node("rc_car", anonymous=True)

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        # Set frequency to 60hz, good for servos.
        self.pwm.set_pwm_freq(60)

        self.__is_loop = True
        self.__speed = 0
        self.__steer = 0
        self.__focus = 0
        self.__zoom = -RcBot.ZOOM_MIN_PULSE
        self.__timestemp = time.time()

        rate = rospy.Rate(10)  # 10hz

        rospy.Subscriber("/cmd_vel", TwistStamped, self.onCmdVel)
        # rospy.Subscriber("/camera/focus_ctrl", TwistStamped, self.on_focus_ctrl)

        sec = 0
        self.setEscPwm(RcBot.PULSE_SPEED_ZERO)

        threading.Thread(target=self.loop, args=()).start()

    def setEscPwm(self, pulse):
        pulse = int(pulse)
        if pulse < RcBot.PULSE_SPEED_ZERO:
            if RcBot.REVERSE_DRIVING_MIN_PULSE < pulse:
                pulse = RcBot.PULSE_SPEED_ZERO
        if pulse > RcBot.PULSE_SPEED_ZERO:
            if RcBot.DRIVING_MIN_PULSE > pulse:
                hpulsez = RcBot.PULSE_SPEED_ZERO
        if RcBot.ESC_MIN_PULSE > pulse:
            pulse = RcBot.PULSE_SPEED_ZERO
        if RcBot.ESC_MAX_PULSE < pulse:
            pulse = RcBot.PULSE_SPEED_ZERO
        try:
            rospy.loginfo("setEscPwm %f" % pulse)
            self.pwm.set_pwm(self.CH_ESC, 0, pulse)
        except:
            traceback.print_exc()

    def on_focus_ctrl(self, msg):
        focus = msg.twist.angular.y * RcBot.FOCUS_SIZE_PER_ONE
        if focus > RcBot.MAX_FOCUS_PULSE_SIZE:
            focus = RcBot.MAX_FOCUS_PULSE_SIZE
        if focus < -RcBot.MAX_FOCUS_PULSE_SIZE:
            focus = -RcBot.MAX_FOCUS_PULSE_SIZE
        self.__focus = focus

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
        try:
            # rospy.loginfo("set_focus_pwm %d" % pulse)
            self.pwm.set_pwm(self.CH_FOCUS, 0, pulse)
        except:
            traceback.print_exc()

    def set_zoom_pwm(self, pulse):
        pulse = int(pulse)
        try:
            # rospy.loginfo("set_zoom_pwm %d" % pulse)
            self.pwm.set_pwm(self.CH_ZOOM, 0, pulse)
        except:
            traceback.print_exc()

    def setSteerPwm(self, pulse):
        pulse = int(pulse)
        try:
            # rospy.loginfo("setSteerPwm %d" % pulse)
            self.pwm.set_pwm(self.CH_SERVO, 0, pulse)
        except:
            traceback.print_exc()

    def loop(self):
        while self.__is_loop:
            if time.time() > self.__timestemp + 0.3:
                if self.__speed != 0:
                    rospy.loginfo("STOP")
                    self.__speed = 0
                    pulse = RcBot.PULSE_SPEED_ZERO + self.__speed
                    self.setEscPwm(pulse)
            time.sleep(0.1)

    def onCmdVel(self, msg):
        # rospy.loginfo(msg)

        steer = msg.twist.angular.z * RcBot.STEER_SIZE_PER_ONE
        if steer > RcBot.MAX_STEER_PULSE_SIZE:
            steer = RcBot.MAX_STEER_PULSE_SIZE
        if steer < -RcBot.MAX_STEER_PULSE_SIZE:
            steer = -RcBot.MAX_STEER_PULSE_SIZE
        self.__steer = steer

        pulse = RcBot.STEER_CENTER_PULSE + steer
        self.setSteerPwm(pulse)

        if msg.twist.linear.x > 0:
            speed = msg.twist.linear.x * RcBot.ACCEL_SIZE_PER_ONE
        elif msg.twist.linear.x < 0:
            speed = msg.twist.linear.x * RcBot.REVERSE_ACCEL_SIZE_PER_ONE
        else:
            speed = 0
        self.__speed = speed

        pulse = RcBot.PULSE_SPEED_ZERO + speed
        self.setEscPwm(pulse)

        self.__timestemp = time.time()

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
