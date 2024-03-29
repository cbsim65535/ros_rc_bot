#!/usr/bin/python

# Python library to interface with the chip LS7366R for the Raspberry Pi
# Written by Federico Bolanos
# Last Edit: May 12th 2020
# Reason: Updating to python3... better late than never eh?

import spidev
from time import sleep


# Usage: import LS7366R then create an object by calling enc = LS7366R(CSX, CLK, BTMD)
# CSX is either CE0 or CE1, CLK is the speed, BTMD is the bytemode 1-4 the resolution of your counter.
# example: lever.Encoder(0, 1000000, 4)
# These are the values I normally use.

class LS7366R():

    # -------------------------------------------
    # Constants
    DURATION_REST = 0.01

    #   Commands
    CLEAR_COUNTER = 0x20
    CLEAR_STATUS = 0x30
    READ_COUNTER = 0x60
    READ_STATUS = 0x70
    WRITE_MODE0 = 0x88
    WRITE_MODE1 = 0x90

    #   Modes
    FOURX_COUNT = 0x03

    FOURBYTE_COUNTER = 0x00
    THREEBYTE_COUNTER = 0x01
    TWOBYTE_COUNTER = 0x02
    ONEBYTE_COUNTER = 0x03

    BYTE_MODE = [ONEBYTE_COUNTER, TWOBYTE_COUNTER,
                 THREEBYTE_COUNTER, FOURBYTE_COUNTER]

    #   Values
    max_val = 4294967295

    # Global Variables

    counterSize = 4  # Default 4

    # ----------------------------------------------
    # Constructor

    def __init__(self, CSX, CLK, BTMD):
        self.counterSize = BTMD  # Sets the byte mode that will be used

        self.spi = spidev.SpiDev()  # Initialize object
        self.spi.open(0, CSX)  # Which CS line will be used
        # Speed of clk (modifies speed transaction)
        self.spi.max_speed_hz = CLK

        # Init the Encoder
        print('Clearing Encoder CS{}\'s Count...\t{}'.format(
            CSX, self.clearCounter()))
        print('Clearing Encoder CS{}\'s Status..\t{}'.format(CSX, self.clearStatus))

        self.spi.xfer2([self.WRITE_MODE0, self.FOURX_COUNT])

        sleep(self.DURATION_REST)  # Rest

        self.spi.xfer2([self.WRITE_MODE1, self.BYTE_MODE[self.counterSize-1]])

    def close(self):
        print('\nClosing SPI port')
        self.spi.close()

    def clearCounter(self):
        self.spi.xfer2([self.CLEAR_COUNTER])

        return '[DONE]'

    def clearStatus(self):
        self.spi.xfer2([self.CLEAR_STATUS])

        return '[DONE]'

    def readCounter(self):
        readTransaction = [self.READ_COUNTER]

        for i in range(self.counterSize):
            readTransaction.append(0)

        data = self.spi.xfer2(readTransaction)

        EncoderCount = 0
        for i in range(self.counterSize):
            EncoderCount = (EncoderCount << 8) + data[i+1]

        if data[1] != 255:
            return EncoderCount
        else:
            return (EncoderCount - (self.max_val+1))

    def readStatus(self):
        data = self.spi.xfer2([self.READ_STATUS, 0xFF])

        return data[1]


if __name__ == "__main__":
    from time import sleep

    encoder = LS7366R(0, 1000000, 4)
    try:
        while True:
            print("Encoder count: %d Press CTRL-C to terminate test program." %
                  (encoder.readCounter()))
            sleep(0.02)
    except KeyboardInterrupt:
        encoder.close()
        print("Test programming ending.")
