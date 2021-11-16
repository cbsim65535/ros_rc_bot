#!/usr/bin/python

# ===========================================================================
# Rotary Encoder (Gray Code) Python Library
# Based on Guy Carpenter's Libary rotary_encoder.py
# https://github.com/guyc/py-gaugette
# Author: MakerBro for ACROBOTIC Industries
# Date: 05/08/2016
# ===========================================================================

import RPi.GPIO as GPIO
import math
import threading
import time


class Encoder:
    def __init__(self, a_pin, b_pin):
        self.a_pin = a_pin
        self.b_pin = b_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.a_pin, GPIO.IN)
        GPIO.setup(self.b_pin, GPIO.IN)

        self.last_delta = 0
        self.r_seq = self.rotation_sequence()

        # steps_per_cycle and remainder are only used in get_cycles which
        # returns a coarse-granularity step count.  By default
        # steps_per_cycle is 4 as there are 4 steps per
        # detent on my encoder, and get_cycles() will return -1 or 1
        # for each full detent step.
        self.steps_per_cycle = 4
        self.remainder = 0
        print("*"*50)

    # Returns the quadrature encoder state converted into
    # a numerical sequence 0,1,2,3,0,1,2,3...
    #
    # Turning the encoder clockwise generates these
    # values for switches B and A:
    #  B A B^A B^A|B<<1
    #  0 0  0     0
    #  0 1  1     1
    #  1 1  0     2
    #  1 0  1     3
    def rotation_sequence(self):
        a_state = GPIO.input(self.a_pin)
        b_state = GPIO.input(self.b_pin)
        r_seq = (a_state ^ b_state) | b_state << 1
        # print(a_state, b_state)
        return r_seq

    # Returns offset values of -2,-1,0,1,2
    def get_delta(self):
        delta = 0
        r_seq = self.rotation_sequence()        
        if r_seq != self.r_seq:
            delta = (r_seq - self.r_seq) % 4
            if delta == 3:
                delta = -1
            elif delta == 2:
                delta = int(
                    math.copysign(delta, self.last_delta)
                )  # same direction as previous, 2 steps

            self.last_delta = delta
            self.r_seq = r_seq
        
        return delta

    def get_cycles(self):
        # python negative integers do not behave like they do in C.
        #   -1 // 2 = -1 (not 0)
        #   -1 % 2 =  1 (not -1)
        # // is integer division operator.  Note the behaviour of the / operator
        # when used on integers changed between python 2 and 3.
        # See http://www.python.org/dev/peps/pep-0238/
        self.remainder += self.get_delta()
        cycles = self.remainder // self.steps_per_cycle
        self.remainder %= self.steps_per_cycle  # remainder always remains positive
        return cycles


cycles = 0
t = time.time()
if __name__ == "__main__":
    # encoder = Encoder(13, 15) #left #0
    encoder = Encoder(16, 18) #right #1
    while True:
        tmp = encoder.get_cycles()
        if tmp == 0:
            continue
        if (tmp * cycles) < 0:
            t = time.time()
            cycles = 0
        cycles += tmp
        print "%d cycles in %f" % (cycles, time.time() - t)

