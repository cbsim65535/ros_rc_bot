# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
# import logging
# logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
# pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 409  # Min pulse length out of 4096
servo_max = 3686  # Max pulse length out of 4096
servo_mid = 2048  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

print("Moving servo on channel 0, press Ctrl-C to quit...")
while True:
    # Move servo on channel O between extremes.
    i = input("0 = middle, 1 = max, 2 = min")
    if i == 0:
        print(servo_mid)
        pwm.set_pwm(1, 0, servo_mid)
    elif i == 1:
        print(servo_max)
        pwm.set_pwm(1, 0, servo_max)
    elif i == 2:
        print(servo_min)
        pwm.set_pwm(1, 0, servo_min)
    time.sleep(1)
