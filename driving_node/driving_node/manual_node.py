#!/usr/bin/env python

import rclpy
import threading
import traceback
import time

from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

from basecam.srv import (
    BasecamSetMotor,
    BasecamSetMotorRequest,
    BasecamResetFollowOffset,
    BasecamResetFollowOffsetRequest,
)


class ManualNode(Node):
    def __init__(self):
        super().__init__("manual_node")
        self.is_loop = True

        self.is_send_base = False
        self.is_send_camera = False
        self.is_send_focus = False

        self.base_twist = Twist()
        self.camera_twist = Twist()
        self.focus_twist = Twist()

        self.sub_joy = self.create_subscription(Joy, "/joy", self.on_cmd_vel, 10)

        self.pub_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.pub_camera_ctrl = self.create_publisher(
            TwistStamped, "/basecam/direct_ctrl", 10
        )
        self.pub_focus_ctrl = self.create_publisher(
            TwistStamped, "/camera/focus_ctrl", 10
        )

        self.basecam_set_moter_client = self.create_client(
            BasecamSetMotor, "/basecam/set_motor"
        )
        self.basecam_reset_follow_offset_client = self.create_client(
            BasecamResetFollowOffset, "/basecam/reset_follow_offset"
        )

        threading.Thread(
            target=self.loop,
            args=(),
        ).start()

    def on_joy(self, msg):
        self.is_send_base = False
        self.is_send_camera = False
        self.is_send_focus = False

        if msg.buttons[4] == 0 and msg.buttons[5] == 1:
            self.is_send_base = True
            if msg.axes[1] > 0:
                self.base_twist.linear.x = msg.axes[1] * 1.0
            else:
                self.base_twist.linear.x = msg.axes[1] * 1.0
            self.base_twist.angular.z = msg.axes[2] * 0.7

        elif msg.buttons[4] == 1 and msg.buttons[5] == 0:
            self.is_send_camera = True
            if msg.buttons[0]:
                req = BasecamSetMotorRequest()
                req.power = True
                self.basecam_set_moter_proxy(req)
            if msg.buttons[3]:
                req = BasecamSetMotorRequest()
                req.power = False
                self.basecam_set_moter_proxy(req)
            if msg.buttons[8]:
                req = BasecamResetFollowOffsetRequest()
                self.basecam_reset_follow_offset_proxy(req)
            self.camera_twist.angular.x = msg.axes[1] * 1.0
            self.camera_twist.angular.z = msg.axes[0] * 1.0
            self.camera_twist.angular.y = msg.axes[2] * 1.0

        # elif msg.buttons[4] == 0 and msg.buttons[5] == 0:
        #     self.is_send_focus = True
        #     self.focus_twist.angular.y = msg.axes[5] * 1.0
        #     self.focus_twist.angular.x = msg.axes[1] * 1.0

    def loop(self):
        while self.is_loop:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()

            if self.is_send_base:
                twist_stapmed = TwistStamped(header=header, twist=self.base_twist)
                self.pub_cmd_vel.publish(twist_stapmed)
            else:
                twist_stapmed = TwistStamped(header=header, twist=Twist())
                self.pub_cmd_vel.publish(twist_stapmed)

            if self.is_send_camera:
                twist_stapmed = TwistStamped(header=header, twist=self.camera_twist)
                self.pub_camera_ctrl.publish(twist_stapmed)
            else:
                twist_stapmed = TwistStamped(header=header, twist=Twist())
                self.pub_camera_ctrl.publish(twist_stapmed)

            if self.is_send_focus:
                twist_stapmed = TwistStamped(header=header, twist=self.focus_twist)
                self.pub_focus_ctrl.publish(twist_stapmed)
            else:
                twist_stapmed = TwistStamped(header=header, twist=Twist())
                self.pub_focus_ctrl.publish(twist_stapmed)

            time.sleep(0.1)

    def stop(self):
        self.is_loop = False


def main(args=None):
    rclpy.init(args=args)
    manual = ManualNode()
    rclpy.spin(manual)
    manual.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
