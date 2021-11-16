#!/usr/bin/python
# -*- coding: utf8 -*-#
#


import pigpio
import time

pi=pigpio.pi()
print(pi)
print(pi.get_hardware_revision())

print("1")
pi.hardware_PWM(18, 100, 160000)
time.sleep(2)
print("2")
pi.hardware_PWM(18, 100, 134000)
time.sleep(2)
print("3")
pi.hardware_PWM(18, 100, 147000)
time.sleep(2)
