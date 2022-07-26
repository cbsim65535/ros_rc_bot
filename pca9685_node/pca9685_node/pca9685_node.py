#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from pwm_msgs.msg import PwmCtrl

import traceback
import time

# Import the PCA9685 module.
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685


class PCA9685Node(Node):
    def __init__(self):
        super().__init__("pca9685_node")

        # Initialise the PCA9685 using the default address (0x40).
        i2c_bus = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(i2c_bus)

        # Set frequency to 60hz, good for servos.
        self.pwm.frequency = 60

        self.__timestamp = [0, 0, 0, 0]
        self.__value = [0, 0, 0, 0]
        self.MAX_CHANNEL = 4

        self.subscription_pwm_ctrl_set = self.create_subscription(
            PwmCtrl, "/pwm_ctrl/set", self.on_ctrl_set, 10
        )
        self.subscription_pwm_ctrl_set  # prevent unused variable warning

        self.__neutral = [369, 2048, 369, 369]

        for i in range(0, self.MAX_CHANNEL):
            self.set_pwm(i, self.__neutral[i])

        rate = self.create_rate(10)

        try:
            while rclpy.ok():
                for i in range(0, self.MAX_CHANNEL):
                    if (
                        time.time() > self.__timestamp[i] + 0.3
                        and self.__value[i] != self.__neutral[i]
                    ):
                        self.set_pwm(i, self.__neutral[i])
                rate.sleep()
        except KeyboardInterrupt:
            traceback.print_exc()

    def on_ctrl_set(self, msg):
        self.set_pwm(msg.channel, msg.pulse)

    def set_pwm(self, channel, pulse):
        self.__timestamp[channel] = time.time()
        pulse = int(pulse)
        self.__value[channel] = pulse
        self.pwm.channels[0].duty_cycle = pulse  # 12bit -> 16bit


def main(args=None):
    rclpy.init(args=args)
    pca9685 = PCA9685Node()
    rclpy.spin(pca9685)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
