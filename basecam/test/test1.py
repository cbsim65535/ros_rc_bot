#!/usr/bin/python
# -*- coding: utf8 -*-#

import serial
import sys
import signal
import time
import struct
import random
import threading
from pylink import link_from_url

CMD_DEF = {
    "CMD_BOARD_INFO"
}

''' protocal 2.6x '''


class SimpleBGC:

    ANGLE_UNIT = 0.02197265625

    class Angle:
        ANGLE_UNIT = 0.02197265625
        roll = 0
        pitch = 0
        yaw = 0

        def __str__(self):
            return "Angle (%d,%d,%d) (%d,%d,%d)" % (self.roll * self.ANGLE_UNIT, self.pitch * self.ANGLE_UNIT, self.yaw * self.ANGLE_UNIT, self.roll , self.pitch, self.yaw)

    angle = Angle()    
    PORT = 'serial:/dev/ttySBGC:115200:8N1'
    is_live = True
    link = None
    RESP_DEF = {
        'CMD_BOARD_INFO':{
            'code': 86, 'fmt':'BHBHBI7s',
            'keywords':['BOARD_VER', 'FIRMWARE_VER', 'STATE_FLAGS1', 'BOARD_FEATURES', 'CONNECTION_FLAG', 'FRW_EXTRA_ID', 'RESERVED']},
        'CMD_GET_ANGLES':{
            'code': 73, 'fmt':'<9h',
            'keywords':['ROLL_IMU_ANGLE', 'ROLL_TARGET_ANGLE', 'ROLL_TARGET_SPEED', 'PITCH_IMU_ANGLE', 'PITCH_TARGET_ANGLE', 'PITCH_TARGET_SPEED', 'YAW_IMU_ANGLE', 'YAW_TARGET_ANGLE', 'YAW_TARGET_SPEED']}
    }

    def __init__(self):
        
        self.link = link_from_url(self.PORT)
        t0 = threading.Thread(target=self.updateAngle, args=())
        t0.start()
        t1 = threading.Thread(target=self.listen, args=())
        t1.start()

    def _checksum8bytes(self, string):
        '''Returns checksum  value from string.'''
        checksum = 0
        # for each char in the string
        for ch in string:
            try:
                c = ord(ch)
            except:
                c = ch
            checksum = (checksum + c) & 0xFF
        return chr(checksum)

    def str2Hex(self, string):
        return ':'.join(x.encode('hex') for x in string);

    def getDefine(self, code):
        for i in self.RESP_DEF:
            item = self.RESP_DEF[i]
            if code == item['code']:
                return item
        return None

    def unpack(self, bytes):        
        r = {}
        # print self.str2Hex(bytes)
        # print ord(bytes[1])        
        if self.isPackage(bytes):
            size = ord(bytes[2])
            define = self.getDefine(ord(bytes[1]))
            t0 = struct.unpack(define['fmt'], bytes[4:size + 4])
            r = dict(zip(define['keywords'], t0))
            return r
        else :
            return None
        return None

    def isPackage(self, bytes):
        r = False
        if len(bytes) < 5:
            return False
        if bytes[0] != chr(0x3e):
            return False
        if bytes[1] == chr(0x56):
            r = True
        if bytes[1] == chr(0x49):
            r = True
        return r

    def pack(self, header, body):
        body_size = 0
        if body is None:
            body = ""
            body_size = 0
        else:
            body_size = len(body)
        # print body_size
        header_pack = header + chr(body_size)    
        result = chr(0x3e) + header_pack + self._checksum8bytes(header_pack) + body + self._checksum8bytes(body)
        # print self.str2Hex(result)
        return result

    '''CMD_BOARD_INFO'''

    def cmdBoradInfo(self):
        header = chr(86)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    '''CMD_CONTROL'''

    def cmdControlModeAngleDegree(self, roll, pitch, yaw):
        roll = round(roll / self.ANGLE_UNIT)
        pitch = round(pitch / self.ANGLE_UNIT)
        yaw = round(yaw / self.ANGLE_UNIT)
        # print roll, pitch, yaw
        header = chr(67)
        body = struct.pack("<3B6h", 2, 2, 2, 0, roll, 0, pitch, 0, yaw)[:15]
        values = self.pack(header, body)
        # print self.str2Hex(values)
        self.link.write(values)

    '''CMD_CONTROL'''

    def cmdControlModeSpeedAngleDegree(self, roll_speed, roll_angle, pitch_speed, pitch_angle, yaw_speed, yaw_angle):
        roll_speed = round(roll_speed / 0.1220740379)
        roll_angle = round(roll_angle / self.ANGLE_UNIT)
        pitch_speed = round(pitch_speed / 0.1220740379)
        pitch_angle = round(pitch_angle / self.ANGLE_UNIT)
        yaw_speed = round(yaw_speed / 0.1220740379)
        yaw_angle = round(yaw_angle / self.ANGLE_UNIT)
        print  roll_angle, pitch_angle, yaw_angle
        # print roll_speed, roll_angle, pitch_speed, pitch_angle, yaw_speed, yaw_angle
        header = chr(67)
        body = struct.pack("<3B6h", 3, 3, 3, roll_speed, roll_angle, pitch_speed, pitch_angle, yaw_speed, yaw_angle)[:15]
        values = self.pack(header, body)
        # print self.str2Hex(values)
        self.link.write(values)

    '''CMD_MOTORS_ON'''

    def cmdMotorsOn(self):
        header = chr(77)
        body = None
        values = self.pack(header, body)
        # print self.str2Hex(values)
        self.link.write(values)

    '''CMD_MOTORS_ON'''

    def cmdMotorsOff(self):
        header = chr(109)
        body = None
        values = self.pack(header, body)
        # print self.str2Hex(values)
        self.link.write(values)

    '''CMD_GET_ANGLES'''

    def cmdGetAngles(self):
        header = chr(73)
        body = None
        values = self.pack(header, body)
        # print self.str2Hex(values)
        self.link.write(values)

    def updateAngle(self):
        while self.is_live:
            self.cmdGetAngles()
            time.sleep(0.1)
    
    def listen(self):
        while self.is_live: 
            b = self.link.read()           
            r = self.unpack(b)
            if r and ord(b[1]) == self.RESP_DEF["CMD_GET_ANGLES"]['code']:
                self.angle.roll = r['ROLL_IMU_ANGLE']
                self.angle.pitch = r['PITCH_IMU_ANGLE']
                self.angle.yaw = r['YAW_IMU_ANGLE']
                print self.angle
            # time.sleep(0.1)

        
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sbgc.is_live = False
    sbgc.cmdMotorsOff()
    exit(0)


if __name__ == "__main__" :
    signal.signal(signal.SIGINT, signal_handler)
    sbgc = SimpleBGC()            
    sbgc.cmdMotorsOn()
    sbgc.cmdControlModeSpeedAngleDegree(0, 0, 0, 0, 0, 0)
    time.sleep(5)
    y = 0
    p = 0
    r = 0
    i = 0
    t = 0    
    while True:
        try:
            i = i + 1
            if i > t:
                y = 0
                p = 0
                r = 0
                y = random.random() * 100 - 50
                p = -random.random() * 30 - 10
                r = random.random() * 30 - 15
                s = random.random() * 50 + 5
                sbgc.cmdControlModeSpeedAngleDegree(s, r, s, p, s, y)
                print r, p, y
                t = round(random.random() * 3) + 5
                i = 0
            # cmdMotorsOff()
            # cmdBoradInfo()
            # size = 0
            # header = chr(86)+chr(size)
            # body = chr(0)
            # values = chr(0x3e)+header+chr(_checksum8bytes(header))+body
            # link.write(values)
            # print str2Hex(values)
        except serial.SerialException as e:
            print e
            break
        time.sleep(1)
