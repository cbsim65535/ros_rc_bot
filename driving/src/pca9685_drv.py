#!/usr/bin/env python

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
        self.__timestemp_0 = time.time()
        self.__timestemp_1 = time.time()
        self.__timestemp_2 = time.time()
        self.__timestemp_3 = time.time()

        rate = rospy.Rate(10)  # 10hz

        rospy.Subscriber("/pwm_ctrl/set", PwmCtrl, self.on_ctrl_set)

        self.neutral_0 = 369
        self.neutral_1 = 2048
        self.neutral_2 = 369
        self.neutral_3 = 369

        self.set_pwm(0, self.neutral_0)
        self.set_pwm(0, self.neutral_1)
        self.set_pwm(0, self.neutral_2)
        self.set_pwm(0, self.neutral_3)

        threading.Thread(target=self.loop, args=()).start()

    def on_ctrl_set(self, msg):
        self.set_pwm(msg.channel, msg.pulse)

    def set_pwm(self, channel, pulse):
        if channel == 0:
            self.__timestemp_0 = time.time()
        if channel == 1:
            self.__timestemp_1 = time.time()
        if channel == 2:
            self.__timestemp_2 = time.time()
        if channel == 3:
            self.__timestemp_3 = time.time()
        pulse = int(pulse)
        self.pwm.set_pwm(channel, 0, pulse)

    def loop(self):
        while self.__is_loop:
            if time.time() > self.__timestemp_0 + 0.3:
                self.set_pwm(0, self.neutral_0)
                pass
            if time.time() > self.__timestemp_1 + 0.3:
                self.set_pwm(0, self.neutral_1)
            if time.time() > self.__timestemp_2 + 0.3:
                # self.set_pwm(0, self.neutral_2)
                pass
            if time.time() > self.__timestemp_3 + 0.3:
                # self.set_pwm(0, self.neutral_3)
                pass
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
