#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import traceback
import threading
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

        self.subscription_pwm_ctrl_0 = self.create_subscription(
            Int32, "/pwm_ctrl/set/0", self.on_ctrl_0, 10
        )
        self.subscription_pwm_ctrl_0

        self.subscription_pwm_ctrl_1 = self.create_subscription(
            Int32, "/pwm_ctrl/set/1", self.on_ctrl_1, 10
        )
        self.subscription_pwm_ctrl_1

        self.__neutral = [369, 2048, 369, 369]

        for i in range(0, self.MAX_CHANNEL):
            self.set_pwm(i, self.__neutral[i])

        self.rate = self.create_rate(10)

        t0 = threading.Thread(target=self.loop, args=())
        t0.start()

    def loop(self):
        try:
            while rclpy.ok():
                for i in range(0, self.MAX_CHANNEL):
                    if (
                        time.time() > self.__timestamp[i] + 0.3
                        and self.__value[i] != self.__neutral[i]
                    ):
                        self.set_pwm(i, self.__neutral[i])
                self.rate.sleep()
        except KeyboardInterrupt:
            traceback.print_exc()

    def on_ctrl_0(self, msg):
        self.set_pwm(0, msg.data)

    def on_ctrl_1(self, msg):
        self.set_pwm(1, msg.data)

    def set_pwm(self, channel, pulse):
        self.__timestamp[channel] = time.time()
        pulse = int(pulse)
        self.__value[channel] = pulse
        self.pwm.channels[channel].duty_cycle = pulse * 16  # 12bit -> 16bit


def main(args=None):
    rclpy.init(args=args)
    pca9685 = PCA9685Node()
    rclpy.spin(pca9685)
    pca9685.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
