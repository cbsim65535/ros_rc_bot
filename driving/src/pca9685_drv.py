#!/usr/bin/env python

from turtle import pu
import rospy
import threading
import traceback
import time

from pwm_msgs.msg import PwmCtrl

# Import the PCA9685 module.
import Adafruit_PCA9685


class RcBot:
    def __init__(self):
        rospy.init_node("pca9685_drv", anonymous=True)

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        # Set frequency to 60hz, good for servos.
        self.pwm.set_pwm_freq(60)

        self.__is_loop = True
        self.__timestamp = [0, 0, 0, 0]
        self.__value = [0, 0, 0, 0]
        self.MAX_CHANNEL = 4

        rate = rospy.Rate(10)  # 10hz

        rospy.Subscriber("/pwm_ctrl/set", PwmCtrl, self.on_ctrl_set)

        self.__neutral = [369, 2048, 369, 369]

        for i in range(0, self.MAX_CHANNEL):
            self.set_pwm(i, self.__neutral[i])

        threading.Thread(target=self.loop, args=()).start()

    def on_ctrl_set(self, msg):
        self.set_pwm(msg.channel, msg.pulse)

    def set_pwm(self, channel, pulse):
        rospy.loginfo("=" * 20)
        rospy.loginfo(channel)
        rospy.loginfo(pulse)
        rospy.loginfo(self.__value[channel])
        rospy.loginfo(self.__neutral[channel])
        rospy.loginfo(self.__timestamp[channel])
        self.__timestamp[channel] = time.time()
        pulse = int(pulse)
        self.__value[channel] = pulse
        self.pwm.set_pwm(channel, 0, pulse)

    def loop(self):
        while self.__is_loop:
            for i in range(0, self.MAX_CHANNEL):
                if (
                    time.time() > self.__timestamp[i]
                    and self.__value[i] != self.__neutral[i]
                ):
                    self.set_pwm(i, self.__neutral[i])
            time.sleep(0.1)

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
