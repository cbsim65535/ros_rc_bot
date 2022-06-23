# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import math

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
servo_mid = 1638  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

print("Moving servo on channel 0, press Ctrl-C to quit...")
while True:
    # Move servo on channel O between extremes.
    i = input("0~1000 ?")
    pulse = int(math.floor(float(i) / 1000 * 4096))
    print(pulse)
    pwm.set_pwm(1, 0, pulse)
    time.sleep(1)
