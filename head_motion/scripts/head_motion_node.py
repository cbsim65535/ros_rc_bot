#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import signal
import time
import rospy
import traceback
import threading
import serial
import struct
import random
import math
import tf
import numpy
from pylink import link_from_url
from std_msgs.msg import String, Empty, UInt8, UInt16, Time, Header
from geometry_msgs.msg import Vector3, Vector3Stamped, QuaternionStamped, Quaternion, PoseStamped
from basecam.srv import BasecamChangeAngle, BasecamChangeAngleRequest
from basecam.srv import BasecamSetMotor, BasecamSetMotorRequest
from head_motion.srv import ChangeMotionMode, ChangeMotionModeResponse
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MotionControl:

    PEOPLE_TRACKED_MOTION_DELAY_MIN = 10
    PEOPLE_TRACKED_MOTION_DELAY_MAX = 40
    motion_sleep = 0
    is_live = True

    class Angle:
        roll = 0
        pitch = 0
        yaw = 0

        def __init__(self, roll, pitch, yaw):
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

        def __str__(self):
            return "Angle (%d,%d,%d)" % (self.roll, self.pitch, self.yaw)

    angle = Angle(0, 0, 0)
    target_angle = Angle(0, 0, 0)

    def __init__(self):
        name = rospy.get_param('~name', "head_contoller")
        rospy.Subscriber(rospy.get_param(
            '~topic_angle', 'basecam/angles/euler'), Vector3Stamped, self.onHeadAngle)
        # rospy.Subscriber(rospy.get_param('~topic_object_markers',
                                        #  '/object_detection/markers'), MarkerArray, self.onObjectMarkers)
        rospy.Service(name+"/change_mode", ChangeMotionMode, self.changeMode)
        self.setMotor = rospy.ServiceProxy(
            "/basecam/set_motor", BasecamSetMotor)
        self.changeAngle = rospy.ServiceProxy(
            "/basecam/change_angle", BasecamChangeAngle)
        name = rospy.get_param('~name', "head_motion")
        self.pub_event = rospy.Publisher(name+"/event", String, queue_size=10)

        # param define
        self.tracking_min_pitch = rospy.get_param('~tracking_min_pitch', -40)
        self.tracking_max_pitch = rospy.get_param('~tracking_max_pitch', 10)
        self.auto_min_pitch = rospy.get_param('~auto_min_pitch', -30)
        self.auto_max_pitch = rospy.get_param('~auto_max_pitch', 10)
        self.auto_min_yaw = rospy.get_param('~auto_min_yaw', -45)
        self.auto_max_yaw = rospy.get_param('~auto_max_yaw', 45)
        self.auto_pitch_step = rospy.get_param('~auto_pitch_step', 5)
        self.auto_yaw_step = rospy.get_param('~auto_yaw_step', 10)
        self.auto_pitch_direction = 1
        self.auto_yaw_direction = 1
        self.detect_motion_delay = rospy.get_param('~detect_motion_delay', 40)
        self.nect_height = rospy.get_param('~nect_height', 0.2)
        self.enable_people_tracking_motion = rospy.get_param(
            '~enable_people_tracking_motion', True)
        self.enable_finding_motion = rospy.get_param(
            '~enable_finding_motion', True)
        self.people_tracking_object_min_distance = rospy.get_param(
            '~people_tracking_object_min_distance', 0.2)

        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service("/basecam/set_motor")
        req = BasecamSetMotorRequest()
        req.power = True
        self.setMotor(req)

        rospy.wait_for_service("/basecam/change_angle")
        req = BasecamChangeAngleRequest()
        req.roll_degree = 0
        req.pitch_degree = 0
        req.yaw_degree = 0
        self.changeAngle(req)

        t0 = threading.Thread(target=self.updateAngle, args=())
        t0.start()

    def onHeadAngle(self, msg):
        self.angle.roll = msg.vector.x
        self.angle.pitch = msg.vector.y
        self.angle.yaw = msg.vector.z
        pass

    def copy_header(self, header):
        h = Header()
        h.seq = header.seq
        h.stamp = header.stamp
        h.frame_id = header.frame_id
        return h

    def transform_pose(self, tf_listener, target_frame_id, pose_stamped_msg):
        try:
            pose = PoseStamped()
            pose.header = self.copy_header(pose_stamped_msg.header)
            pose.pose = pose_stamped_msg.pose
            rospy_time = self.tf_listener.getLatestCommonTime(
                target_frame_id, pose.header.frame_id)
            pose.header.stamp = rospy_time
            pose = self.tf_listener.transformPose(target_frame_id, pose)
            pose.header.seq = pose_stamped_msg.header.seq
            return pose
        except:
            traceback.print_exc()
            return None

    def onObjectMarkers(self, msg):
        total_people = 0
        peoples_maker = []
        for marker in msg.markers:
            if marker.text == "face":
                peoples_maker.append(marker)
        total_people = len(peoples_maker)
        if total_people > 0:
            # 최근 추가된 사람을 추적
            marker = peoples_maker[total_people-1]
            if marker.text == "face":
                if self.enable_people_tracking_motion:
                    try:
                        base_frame = '/neck_link'
                        newPose = self.transform_pose(
                            self.tf_listener, base_frame, marker)
                        if newPose:
                            zero = numpy.array((0, 0, 0))
                            tgt = numpy.array(
                                (newPose.pose.position.x, newPose.pose.position.y, newPose.pose.position.z))
                            dist = numpy.linalg.norm(zero-tgt)
                            if dist > self.people_tracking_object_min_distance:
                                dist_xy = abs(math.hypot(
                                    newPose.pose.position.x, newPose.pose.position.y))
                                roll = random.random() * 30 - 15
                                pitch = -math.degrees(math.atan2(
                                    newPose.pose.position.z-self.nect_height, dist_xy))
                                yaw = -math.degrees(math.atan2(
                                    newPose.pose.position.y, newPose.pose.position.x))
                                self.target_angle.yaw = (
                                    self.target_angle.yaw+yaw)*0.5
                                if random.random() < 0.05:
                                    self.target_angle.roll = roll
                                self.target_angle.pitch = (
                                    self.target_angle.pitch+pitch)*0.5
                                if self.target_angle.pitch < self.tracking_min_pitch:
                                    self.target_angle.pitch = self.tracking_min_pitch
                                if self.target_angle.pitch > self.tracking_max_pitch:
                                    self.target_angle.pitch = self.tracking_max_pitch
                                if self.motion_sleep < 0:
                                    size = (
                                        self.PEOPLE_TRACKED_MOTION_DELAY_MAX-self.PEOPLE_TRACKED_MOTION_DELAY_MIN)
                                    self.motion_sleep = self.PEOPLE_TRACKED_MOTION_DELAY_MIN + \
                                        math.floor(size*random.random())
                    except:
                        traceback.print_exc()

    def changeMode(self, request):
        print "changeMode"
        print request
        if request.mode_name == "random":
            pass
        elif request.mode_name == "follow_face":
            pass
        elif request.mode_name == "stop":
            pass

    def updateAngle(self):
        while self.is_live:
            self.motion_sleep = self.motion_sleep - 1
            if self.motion_sleep == 0:
                req = BasecamChangeAngleRequest()
                req.roll_degree = self.target_angle.roll
                req.pitch_degree = self.target_angle.pitch
                req.yaw_degree = self.target_angle.yaw
                req.roll_speed = 50
                req.pitch_speed = 50
                req.yaw_speed = 50
                self.changeAngle(req)
                msg = String()
                msg.data = "face_tracking"
                self.pub_event.publish(msg)
                # print "people"
                # print self.angle
                # print req
            elif self.motion_sleep < -self.detect_motion_delay:
                if self.enable_finding_motion:
                    ty = self.auto_yaw_step*self.auto_yaw_direction
                    yaw = -self.angle.yaw + ty
                    tp = self.auto_pitch_step*self.auto_pitch_direction
                    pitch = self.angle.pitch + tp
                    if yaw < self.auto_min_yaw:
                        yaw = self.auto_min_yaw
                        self.auto_yaw_direction = self.auto_yaw_direction*-1
                    if yaw > self.auto_max_yaw:
                        yaw = self.auto_max_yaw
                        self.auto_yaw_direction = self.auto_yaw_direction*-1
                    if pitch < self.auto_min_pitch:
                        pitch = self.auto_min_pitch
                        self.auto_pitch_direction = self.auto_pitch_direction*-1
                    if pitch > self.auto_max_pitch:
                        pitch = self.auto_max_pitch
                        self.auto_pitch_direction = self.auto_pitch_direction*-1
                    self.motion_sleep = -1
                    req = BasecamChangeAngleRequest()
                    req.roll_degree = 0
                    req.pitch_degree = pitch
                    req.yaw_degree = yaw
                    req.roll_speed = 50
                    req.pitch_speed = 50
                    req.yaw_speed = 50
                    self.changeAngle(req)
                    msg = String()
                    msg.data = "scanning"
                    self.pub_event.publish(msg)
                    # print "random"
                    # print self.angle
                    # print yaw
                    # print pitch
                    # print req
            time.sleep(0.1)


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    ctrl.is_live = False
    exit(0)


is_live = True
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    name = rospy.get_param('~name', "head_motion")
    rospy.init_node(name, anonymous=True)
    try:
        ctrl = MotionControl()
        rospy.spin()
    except Exception:
        traceback.print_exc()
    except rospy.ROSInterruptException:
        traceback.print_exc()
        pass
