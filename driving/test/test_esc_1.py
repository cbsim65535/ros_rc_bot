#!/usr/bin/python
# -*- coding: utf8 -*-#
#


import pigpio
import time

pi=pigpio.pi()
print(pi)
print(pi.get_hardware_revision())

print("1")
print pi.hardware_PWM(19, 100, 400000)
