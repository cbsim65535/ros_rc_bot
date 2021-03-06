#!/usr/bin/python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tf2_ros import TransformBroadcaster


import time
import traceback
import threading
import struct
import math

from pylink import link_from_url

from geometry_msgs.msg import (
    Vector3,
    Vector3Stamped,
    QuaternionStamped,
    Quaternion,
    TwistStamped,
)
from basecam_msgs.srv import (
    BasecamChangeAngle,
    BasecamSetMotor,
    BasecamResetFollowOffset,
)
from basecam_msgs.action import BasecamDirectAngle
from sensor_msgs.msg import Imu


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class ResponseDefine:

    _define = {
        "CMD_BOARD_INFO": {
            "code": 86,
            "formats": [
                {"BOARD_VER": "B"},
                {"FIRMWARE_VER": "H"},
                {"STATE_FLAGS1": "B"},
                {"BOARD_FEATURES": "H"},
                {"CONNECTION_FLAG": "B"},
                {"FRW_EXTRA_ID": "I"},
                {"RESERVED": "7s"},
            ],
        },
        "CMD_GET_ANGLES": {
            "code": 73,
            "formats": [
                {"ROLL_IMU_ANGLE": "h"},
                {"ROLL_TARGET_ANGLE": "h"},
                {"ROLL_TARGET_SPEED": "h"},
                {"PITCH_IMU_ANGLE": "h"},
                {"PITCH_TARGET_ANGLE": "h"},
                {"PITCH_TARGET_SPEED": "h"},
                {"YAW_IMU_ANGLE": "h"},
                {"YAW_TARGET_ANGLE": "h"},
                {"YAW_TARGET_SPEED": "h"},
            ],
        },
        "CMD_HELPER_DATA": {
            "code": 72,
            "formats": [
                {"FRAME_ACC_ROLL": "h"},
                {"FRAME_ACC_PITCH": "h"},
                {"FRAME_ACC_YAW": "h"},
                {"FRAME_ANGLE_ROLL": "h"},
                {"FRAME_ANGLE_PITCH": "h"},
                {"FLAGS": "B"},
                {"FRAME_SPEED_ROLL": "h"},
                {"FRAME_SPEED_PITCH": "h"},
                {"FRAME_SPEED_YAW": "h"},
                {"FRAME_HEADING": "h"},
                {"RESERVED": "B"},
            ],
        },
        "CMD_REALTIME_DATA_3": {
            "code": 23,
            "formats": [
                {"ACC_DATA_X": "h"},
                {"GYRO_DATA_X": "h"},
                {"ACC_DATA_Y": "h"},
                {"GYRO_DATA_Y": "h"},
                {"ACC_DATA_Z": "h"},
                {"GYRO_DATA_Z": "h"},
                {"SERIAL_ERR_CNT": "H"},
                {"SYSTEM_ERROR": "H"},
                {"SYSTEM_SUB_ERROR": "B"},
                {"RESERVED": "3s"},
                {"RC_ROLL": "h"},
                {"RC_PITCH": "h"},
                {"RC_YAW": "h"},
                {"RC_CMD": "h"},
                {"EXT_FC_ROLL": "h"},
                {"EXT_FC_PITCH": "h"},
                {"IMU_ANGLE_ROLL": "h"},
                {"IMU_ANGLE_PITCH": "h"},
                {"IMU_ANGLE_YAW": "h"},
                {"FRAME_IMU_ANGLE_ROLL": "h"},
                {"FRAME_IMU_ANGLE_PITCH": "h"},
                {"FRAME_IMU_ANGLE_YAW": "h"},
                {"TARGET_ANGLE_ROLL": "h"},
                {"TARGET_ANGLE_PITCH": "h"},
                {"TARGET_ANGLE_YAW": "h"},
                {"CYCLE_TIME": "H"},
                {"I2C_ERROR_COUNT": "H"},
                {"ERROR_CODE": "B"},
                {"BAT_LEVEL": "H"},
                {"RT_DATA_FLAGS": "B"},
                {"CUR_IMU": "B"},
                {"CUR_PROFILE": "B"},
                {"MOTOR_POWER_0": "B"},
                {"MOTOR_POWER_1": "B"},
            ],
        },
        "CMD_READ_PARAMS_3": {  # this command enable firmware version ??
            "code": 21,
            "formats": [
                {"PROFILE_ID": "B"},
                {"ROLL_P": "B"},
                {"ROLL_I": "B"},
                {"ROLL_D": "B"},
                {"ROLL_POWER": "B"},
                {"ROLL_INVERT": "B"},
                {"ROLL_POLES": "B"},
                {"PITCH_P": "B"},
                {"PITCH_I": "B"},
                {"PITCH_D": "B"},
                {"PITCH_POWER": "B"},
                {"PITCH_INVERT": "B"},
                {"PITCH_POLES": "B"},
                {"YAW_P": "B"},
                {"YAW_I": "B"},
                {"YAW_D": "B"},
                {"YAW_POWER": "B"},
                {"YAW_INVERT": "B"},
                {"YAW_POLES": "B"},
                {"ACC_LIMITER_ALL": "B"},
                {"EXT_FC_GAIN_0": "b"},
                {"EXT_FC_GAIN_1": "b"},
                {"ROLL_RC_MIN_ANGLE": "h"},
                {"ROLL_RC_MAX_ANGLE": "h"},
                {"ROLL_RC_MODE": "B"},
                {"ROLL_RC_LPF": "B"},
                {"ROLL_RC_SPEED": "B"},
                {"ROLL_RC_FOLLOW": "B"},
                {"PITCH_RC_MIN_ANGLE": "h"},
                {"PITCH_RC_MAX_ANGLE": "h"},
                {"PITCH_RC_MODE": "B"},
                {"PITCH_RC_LPF": "B"},
                {"PITCH_RC_SPEED": "B"},
                {"PITCH_RC_FOLLOW": "B"},
                {"YAW_RC_MIN_ANGLE": "h"},
                {"YAW_RC_MAX_ANGLE": "h"},
                {"YAW_RC_MODE": "B"},
                {"YAW_RC_LPF": "B"},
                {"YAW_RC_SPEED": "B"},
                {"YAW_RC_FOLLOW": "B"},
                {"GYRO_TRUST": "B"},
                {"USE_MODEL": "B"},
                {"PWM_FREQ": "B"},
                {"SERIAL_SPEED": "B"},
                {"RC_TRIM_0": "b"},
                {"RC_TRIM_1": "b"},
                {"RC_TRIM_2": "b"},
                {"RC_DEADBAND": "B"},
                {"RC_EXPO_RATE": "B"},
                {"RC_VIRT_MODE": "B"},
                {"RC_MAP_ROLL": "B"},
                {"RC_MAP_PITCH": "B"},
                {"RC_MAP_YAW": "B"},
                {"RC_MAP_CMD": "B"},
                {"RC_MAP_FC_ROLL": "B"},
                {"RC_MAP_FC_PITCH": "B"},
                {"RC_MIX_FC_ROLL": "B"},
                {"RC_MIX_FC_PITCH": "B"},
                {"FOLLOW_MODE": "B"},
                {"FOLLOW_DEADBAND": "B"},
                {"FOLLOW_EXPO_RATE": "B"},
                {"FOLLOW_OFFSET_0": "b"},
                {"FOLLOW_OFFSET_1": "b"},
                {"FOLLOW_OFFSET_2": "b"},
                {"AXIS_TOP": "b"},
                {"AXIS_RIGHT": "b"},
                {"FRAME_AXIS_TOP": "b"},
                {"FRAME_AXIS_RIGHT": "b"},
                {"FRAME_IMU_POS": "B"},
                {"GYRO_DEADBAND": "B"},
                {"GYRO_SENS": "B"},
                {"I2C_SPEED_FAST": "B"},
                {"SKIP_GYRO_CALIB": "B"},
                {"RC_CMD_LOW": "B"},
                {"RC_CMD_MID": "B"},
                {"RC_CMD_HIGH": "B"},
                {"MENU_CMD_1": "B"},
                {"MENU_CMD_2": "B"},
                {"MENU_CMD_3": "B"},
                {"MENU_CMD_4": "B"},
                {"MENU_CMD_5": "B"},
                {"MENU_CMD_LONG": "B"},
                {"MOTOR_OUTPUT_0": "B"},
                {"MOTOR_OUTPUT_1": "B"},
                {"MOTOR_OUTPUT_2": "B"},
                {"BAT_THRESHOLD_ALARM": "h"},
                {"BAT_THRESHOLD_MOTOR": "h"},
                {"BAT_COMP_REF": "h"},
                {"BEEPER_MODES": "B"},
                {"FOLLOW_ROLL_MIX_STAR": "B"},
                {"FOLLOW_ROLL_MIX_RANGE": "B"},
                {"BOOSTER_POWER_0": "B"},
                {"BOOSTER_POWER_1": "B"},
                {"BOOSTER_POWER_2": "B"},
                {"FOLLOW_SPEED_0": "B"},
                {"FOLLOW_SPEED_1": "B"},
                {"FOLLOW_SPEED_2": "B"},
                {"FRAME_ANGLE_FROM_MOTORS": "B"},
                {"RC_MEMORY_0": "h"},
                {"RC_MEMORY_1": "h"},
                {"RC_MEMORY_2": "h"},
                {"SERVO1_OUT": "B"},
                {"SERVO2_OUT": "B"},
                {"SERVO3_OUT": "B"},
                {"SERVO4_OUT": "B"},
                {"SERVO_RATE": "B"},
                {"ADAPTIVE_PID_ENABLED": "B"},
                {"ADAPTIVE_PID_THRESHOLD": "B"},
                {"ADAPTIVE_PID_RATE": "B"},
                {"ADAPTIVE_PID_RECOVERY_FACTOR": "B"},
                {"FOLLOW_LPF_0": "B"},
                {"FOLLOW_LPF_1": "B"},
                {"FOLLOW_LPF_2": "B"},
                {"SPEKTRUM_MODE": "B"},
                {"CUR_IMU": "B"},
                {"CUR_PROFILE_ID": "B"},
            ],
        },
    }

    def __init__(self):
        for i in self._define:
            item = self._define[i]
            r = ""
            l = []
            s = len(item["formats"])
            for j in item["formats"]:
                k = list(j.keys())[0]
                r = r + j[k]
                l.append(k)
            item["_fmt"] = r
            item["_keywords"] = l

    def getDefine(self, code):
        for i in self._define:
            item = self._define[i]
            if code == item["code"]:
                return item
        return None

    def getCode(self, command_name):
        for i in self._define:
            if i == command_name:
                return self._define[i]["code"]
        return None

    def hasCode(self, code):
        if self.getDefine(code):
            return True
        return False


class ChangeAngleServer:
    _feedback = BasecamDirectAngle.Feedback()
    _result = BasecamDirectAngle.Result()

    def __init__(self, simplebgc):
        self.is_loop = False
        self.__simplebgc = simplebgc
        self.__server = ActionServer(
            self, BasecamDirectAngle, "basecam/direct_angle", execute_cb=self.execute_cb
        )

    def execute_cb(self, goal):
        r = rclpy.Rate(2)
        rclpy.logging.get_logger().info("direct_angle %r" % (goal))
        self.__simplebgc.cmd_control_mode_speed_angle_degree(
            goal.roll_speed,
            goal.roll_degree,
            goal.pitch_speed,
            goal.pitch_degree,
            goal.yaw_speed,
            goal.yaw_degree,
        )
        self.is_loop = True
        while self.is_loop:
            feedback = BasecamDirectAngleFeedback()
            feedback.roll_degree = self.__simplebgc._now_roll
            feedback.pitch_degree = self.__simplebgc._now_pitch
            feedback.yaw_degree = self.__simplebgc._now_yaw
            rclpy.logging.get_logger().info(feedback)
            self.__server.publish_feedback(self._feedback)
            bool1 = True
            dr = abs(goal.roll_degree - self.__simplebgc._now_roll) % 360
            dp = abs(goal.pitch_degree - self.__simplebgc._now_pitch) % 360
            dy = abs(goal.yaw_degree - self.__simplebgc._now_yaw) % 360
            if dr > 1:
                bool1 = False
            if dp > 1:
                bool1 = False
            if dy > 1:
                bool1 = False
            rclpy.logging.get_logger().info("(%d,%d,%d,%s)" % (dr, dp, dy, str(bool1)))
            if bool1:
                break
            r.sleep()
        self.__server.set_succeeded(self._result)


class Basecam(Node):
    RESP_DEF = ResponseDefine()
    ANGLE_UNIT = 0.02197265625
    ACCEL_UNIT = 0.019160156
    GYRO_UNIT = 0.06103701895
    SPEED_UNIT = 0.1220740379
    PORT = "serial:/dev/ttyUSB0:115200:8N1"

    is_live = True
    link = None
    motion_sleep = 0

    def __init__(self):
        super().__init__("basecam")
        self._now_roll = 0
        self._now_pitch = 0
        self._now_yaw = 0

        self.camera_tf_broadcaster = TransformBroadcaster()

        self.pub_angles_euler = self.create_publisher(
            Vector3Stamped, "/basecam/angles/euler", 10
        )
        self.pub_angles_euler = self.create_publisher(
            QuaternionStamped, "/basecam/angles/quaternion", 10
        )
        self.pub_angles_euler = self.create_publisher(
            Imu, "/basecam/camera/imu/data", 10
        )
        self.pub_angles_euler = self.create_publisher(
            Imu, "/basecam/camera/imu/raw", 10
        )
        self.pub_angles_euler = self.create_publisher(
            Imu, "/basecam/camera/imu/mag", 10
        )
        self.pub_angles_euler = self.create_publisher(
            Imu, "/basecam/frame/imu/data", 10
        )

        self.srv_change_angle = self.create_service(
            BasecamChangeAngle, "/basecam/change_angle", self.change_angle
        )
        self.srv_change_angle = self.create_service(
            BasecamSetMotor, "/basecam/set_motor", self.set_motor
        )
        self.srv_change_angle = self.create_service(
            BasecamResetFollowOffset,
            "/basecam/reset_follow_offset",
            self.reset_follow_offset,
        )

        self.sub_direct_ctrl = self.create_subscription(
            TwistStamped, "/basecam/direct_ctrl", self.on_direct_ctrl, 10
        )

        self.server = ChangeAngleServer(self)

        self.min_pitch = rclpy.get_param("~min_pitch", -40)
        self.max_pitch = rclpy.get_param("~max_pitch", 10)

        self.link = link_from_url(self.PORT)

        t0 = threading.Thread(target=self.updateAngle, args=())
        t0.start()
        t1 = threading.Thread(target=self.listen, args=())
        t1.start()

    def on_direct_ctrl(self, msg):
        sbgc.cmd_control_mode_remote_control(
            msg.twist.angular.x * 500,
            -msg.twist.angular.y * 500,
            -msg.twist.angular.z * 500,
        )
        return

    def set_motor(self, req):
        if req.power == True:
            self.cmd_motors_on()
        else:
            self.cmd_motors_off()
        res = BasecamSetMotor.Response()
        res.ok = True
        if self.server:
            self.server.is_loop = False
        return res

    def reset_follow_offset(self, req):
        self.cmd_calib_offset()
        res = BasecamResetFollowOffset.Response()
        return res

    def change_angle(self, msg):
        rclpy.logging.get_logger().info("change_angle %r" % (msg))
        self.cmd_control_mode_speed_angle_degree(
            msg.roll_speed,
            msg.roll_degree,
            msg.pitch_speed,
            msg.pitch_degree,
            msg.yaw_speed,
            msg.yaw_degree,
        )
        res = BasecamChangeAngle.Response()
        res.ok = True
        return res

    def _checksum8bytes(self, string):
        """Returns checksum  value from string."""
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
        if string is not None:
            return ":".join(x.encode("hex") for x in string)
        else:
            return ""

    def unpack(self, bytes):
        r = {}
        if self.is_package(bytes):
            size = ord(bytes[2])
            define = self.RESP_DEF.getDefine(ord(bytes[1]))
            try:
                body = bytes[4 : size + 4]
                t0 = struct.unpack(define["_fmt"], body)
                r = dict(zip(define["_keywords"], t0))
            except Exception as e:
                traceback.print_exc()
                return None
            return r
        else:
            return None
        return None

    def is_package(self, bytes):
        r = False
        if bytes is None:
            return False
        if len(bytes) < 5:
            return False
        if bytes[0] != chr(0x3E):
            return False
        if self.RESP_DEF.hasCode(ord(bytes[1])):
            r = True
        return r

    def pack(self, header, body):
        body_size = 0
        if body is None:
            body = ""
            body_size = 0
        else:
            body_size = len(body)
        header_pack = header + chr(body_size)
        result = (
            chr(0x3E)
            + header_pack
            + self._checksum8bytes(header_pack)
            + body
            + self._checksum8bytes(body)
        )
        return result

    """CMD_BOARD_INFO"""

    def cmdBoradInfo(self):
        header = chr(86)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_USE_DEFAULTS"""

    def cmdUseDefaults(self):
        header = chr(114)
        body = chr(0)
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_RESET"""

    def cmdReset(self):
        header = chr(114)
        body = struct.pack("<BH", 0, 0)[:3]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_CONTROL"""

    def cmdControlModeAngleDegree(self, roll, pitch, yaw):
        roll = round(roll / self.ANGLE_UNIT)
        pitch = round(pitch / self.ANGLE_UNIT)
        yaw = round(yaw / self.ANGLE_UNIT)
        header = chr(67)
        body = struct.pack("<3B6h", 2, 2, 2, 0, roll, 0, pitch, 0, yaw)[:15]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_CONTROL"""

    def cmd_control_mode_speed_angle_degree(
        self, roll_speed, roll_angle, pitch_speed, pitch_angle, yaw_speed, yaw_angle
    ):
        roll_speed = round(roll_speed / self.SPEED_UNIT)
        roll_angle = round(roll_angle / self.ANGLE_UNIT)
        pitch_speed = round(pitch_speed / self.SPEED_UNIT)
        pitch_angle = round(pitch_angle / self.ANGLE_UNIT)
        yaw_speed = round(yaw_speed / self.SPEED_UNIT)
        yaw_angle = round(yaw_angle / self.ANGLE_UNIT)
        header = chr(67)
        body = struct.pack(
            "<3B6h",
            2,
            2,
            2,
            roll_speed,
            roll_angle,
            pitch_speed,
            pitch_angle,
            yaw_speed,
            yaw_angle,
        )[:15]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_CONTROL
        MODE_RC
    """

    def cmd_control_mode_remote_control(self, roll, pitch, yaw):
        header = chr(67)
        body = struct.pack("<3B6h", 4, 4, 4, 0, roll, 0, pitch, 0, yaw)[:15]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_CALIB_OFFSET"""

    def cmd_calib_offset(self):
        header = chr(79)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_MOTORS_ON"""

    def cmd_motors_on(self):
        header = chr(77)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_MOTORS_ON"""

    def cmd_motors_off(self):
        header = chr(109)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_GET_ANGLES"""

    def cmdGetAngles(self):
        header = chr(73)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_REALTIME_DATA_3"""

    def cmdRealtimeData3(self):
        header = chr(23)
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    def cmdHelperData(self):
        header = chr(self.RESP_DEF.getCode("CMD_HELPER_DATA"))
        body = None
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_DATA_STREAM_INTERVAL"""

    def cmdDataStreamInterval(self, code, ms):
        header = chr(85)
        body = struct.pack("<BH18s", code, ms, "")[:21]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_WRITE_PARAMS"""

    def cmdWriteParams3(self):
        header = chr(22)
        body = struct.pack("<139B", "")[:139]
        body[61]
        values = self.pack(header, body)
        self.link.write(values)

    """CMD_READ_PARAMS_3"""

    def cmdReadParams3(self):
        header = chr(21)
        body = struct.pack("<B", 0)[:1]
        values = self.pack(header, body)
        self.link.write(values)

    def updateAngle(self):
        while self.is_live:
            self.cmdRealtimeData3()
            time.sleep(0.1)
            pass

    def byteJoin(self, b0, b1, b2):
        l = bytearray()
        for i in b0:
            l.append(ord(i))
        for i in b1:
            l.append(ord(i))
        for i in b2:
            l.append(ord(i))
        b = str(l)
        return b

    def listen(self):
        while self.is_live:
            b = None
            b0 = self.link.read(1)
            if b0 == chr(0x3E):
                b1 = self.link.read(2)
                size = ord(b1[1])
                b2 = self.link.read(1 + size + 1)
                b = self.byteJoin(b0, b1, b2)
            r = self.unpack(b)
            if r and ord(b[1]) == self.RESP_DEF.getCode("CMD_REALTIME_DATA_3"):
                roll = -r["IMU_ANGLE_ROLL"] * self.ANGLE_UNIT
                pitch = -r["IMU_ANGLE_PITCH"] * self.ANGLE_UNIT
                yaw = -r["IMU_ANGLE_YAW"] * self.ANGLE_UNIT
                self._now_roll = roll
                self._now_pitch = pitch
                self._now_yaw = yaw
                msg = Vector3Stamped()
                msg.header.stamp = rclpy.Time.now()
                msg.vector = Vector3(roll, pitch, yaw)
                self.pub_angles_euler.publish(msg)
                (x, y, z, w) = quaternion_from_euler(
                    math.radians(roll), math.radians(pitch), math.radians(yaw)
                )
                msg = QuaternionStamped()
                msg.header.stamp = rclpy.Time.now()
                msg.quaternion = Quaternion(x, y, z, w)
                self.pub_angles_quaternion.publish(msg)

                msg = Imu()
                msg.header.stamp = rclpy.Time.now()
                msg.header.frame_id = "camera_mount_imu"
                # IMU_ANGLE_ROLL
                msg.angular_velocity.x = (
                    -r["GYRO_DATA_X"] * self.GYRO_UNIT / 180 * math.pi
                )
                msg.angular_velocity.y = (
                    r["GYRO_DATA_Y"] * self.GYRO_UNIT / 180 * math.pi
                )
                msg.angular_velocity.z = (
                    r["GYRO_DATA_Z"] * self.GYRO_UNIT / 180 * math.pi
                )
                msg.linear_acceleration.x = -r["ACC_DATA_X"] * self.ACCEL_UNIT
                msg.linear_acceleration.y = r["ACC_DATA_Y"] * self.ACCEL_UNIT
                msg.linear_acceleration.z = r["ACC_DATA_Z"] * self.ACCEL_UNIT
                # msg.linear_acceleration.x = 9.81
                msg.orientation = Quaternion(x, y, z, w)
                msg.linear_acceleration_covariance[0] = -1
                msg.angular_velocity_covariance[0] = -1
                msg.orientation_covariance[0] = -1
                self.pub_camera_imu_raw.publish(msg)

                current_time = rclpy.Time.now()
                self.camera_tf_broadcaster.sendTransform(
                    (0.0, 0.0, 0.0),
                    (x, y, z, w),
                    current_time,
                    "gimbal_camera_mount",
                    "gimbal_neck",
                )

            else:
                if b is not None:
                    rclpy.logging.get_logger().debug("result code : %d" % (ord(b[1])))
                    rclpy.logging.get_logger().debug(r)
            # time.sleep(0.1)

    def stop(self):
        self.is_live = False


def main(args=None):
    rclpy.init(args=args)

    basecam = Basecam()

    rclpy.spin(basecam)

    basecam.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
