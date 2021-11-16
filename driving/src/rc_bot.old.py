#!/usr/bin/env python

import rospy
import threading
import traceback
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import pigpio
import time
import traceback
import subprocess
import re
import time

class RcBot:

    GPIO_SERVO = 18
    GPIO_ESC = 19
    SIZE_STEER = 52000
    STEER_CENTER = 147000
    MAX_LEFT = 12000
    MAX_RIGHT = 12000

    ESC_MAX_HZ = 900000
    ESC_MIN_HZ = 100000
    DRIVING_MIN_HZ = 305000
    SIZE_ACCEL = 25000
    REVERSE_DRIVING_MIN_HZ = 225000
    REVERSE_SIZE_ACCEL = 25000
    HZ_SPEED_ZERO = 300000

    def __init__(self):
        rospy.init_node('rc_car', anonymous=True)
        time.sleep(5)
        self.pi = pigpio.pi()
        rospy.loginfo(self.pi)
        self.__is_loop = True
        self.__speed = 0
        self.__steer = 0            
        self.__timestemp = time.time()

        rate = rospy.Rate(10) # 10hz

        rospy.Subscriber("/cmd_vel", Twist, self.onCmdVel)


        sec = 0
        while(sec<5):
            rospy.loginfo(sec)
            cmd = "ps -ef | grep pigpiod | grep -v grep"
            p1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            (result, error) = p1.communicate()
            regex = re.compile(r'\d+:\d+:\d+')
            timestr = regex.search(result).group()
            t = time.strptime(timestr, "%H:%M:%S")
            sec = t.tm_sec + t.tm_min*60 + t.tm_hour*3600
            time.sleep(1)

        self.setEscPwm(RcBot.HZ_SPEED_ZERO)
        
        threading.Thread(target=self.loop, args=()).start()

    def setEscPwm(self,hz):
        rospy.loginfo("setEscPwm")
        hz = int(hz)
        if(hz<RcBot.HZ_SPEED_ZERO):
            if(RcBot.REVERSE_DRIVING_MIN_HZ<hz):
                hz = RcBot.HZ_SPEED_ZERO
        if(hz>RcBot.HZ_SPEED_ZERO):
            if(RcBot.DRIVING_MIN_HZ>hz):
                hz = RcBot.HZ_SPEED_ZERO
        if(RcBot.ESC_MIN_HZ>hz):
            hz = RcBot.HZ_SPEED_ZERO
        if(RcBot.ESC_MAX_HZ<hz):
            hz = RcBot.HZ_SPEED_ZERO 
        rospy.loginfo(hz)
        try:
            self.pi.hardware_PWM(RcBot.GPIO_ESC, 100, hz)
        except:
            traceback.print_exc()


    def setSteerPwm(self,hz):
        hz = int(hz)
        try:
            self.pi.hardware_PWM(RcBot.GPIO_SERVO, 100, hz)
        except:
            traceback.print_exc()
            self.pi = pigpio.pi()
            self.pi.hardware_PWM(RcBot.GPIO_SERVO, 100, hz)

    def loop(self):
        while(self.__is_loop):
            if(time.time()>self.__timestemp+0.3):
                if(self.__speed!=0):
                    rospy.loginfo("STOP")
                    self.__speed = 0
                    hz = RcBot.HZ_SPEED_ZERO + self.__speed
                    self.setEscPwm(hz)
            time.sleep(0.1)


    def onCmdVel(self, msg):
        rospy.loginfo(msg)
        
        steer = msg.angular.z *RcBot.SIZE_STEER
        if(steer > RcBot.MAX_LEFT):
            steer = RcBot.MAX_LEFT
        if(steer < -RcBot.MAX_RIGHT):
            steer = -RcBot.MAX_RIGHT
        self.__steer = steer

        hz = RcBot.STEER_CENTER + steer
        self.setSteerPwm(hz)
                
        if(msg.linear.x>0):
            speed = msg.linear.x * RcBot.SIZE_ACCEL
        elif(msg.linear.x<0):
            speed = msg.linear.x * RcBot.REVERSE_SIZE_ACCEL
        else:
            speed = 0
        self.__speed = speed

        hz = RcBot.HZ_SPEED_ZERO + speed
        self.setEscPwm(hz)
        
        self.__timestemp = time.time()

    def stop(self):
        self.__is_loop = False

if __name__ == '__main__':
    try:
        rcbot = RcBot()
        rospy.spin()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    finally:
        traceback.print_exc()
        rcbot.stop()