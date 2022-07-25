#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading
import time

from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TransformStamped

from bno055 import BNO055

SENSORS_GRAVITY_EARTH = 9.80665


class Manual(Node):
    def __init__(self):
        super().__init__("bno055")

        self.imu_tf_broadcaster = TransformBroadcaster(self)
        self.pub_data = self.create_publisher(Imu, "/mobile/imu/data", 1)
        self.pub_raw = self.create_publisher(Imu, "/mobile/imu/raw", 1)
        self.pub_mag = self.create_publisher(MagneticField, "/mobile/imu/mag", 1)

        self.is_loop = True

        self.bno = BNO055(address=BNO055.BNO055_ADDRESS_B)
        threading.Thread(
            target=self.loop,
            args=(),
        ).start()

    def loop(self):

        acc_fact = 100.0
        mag_fact = 16.0
        gyr_fact = 900.0
        seq = 0
        frame_id = "mobile_imu_base"

        imu_data = Imu()  # Filtered data
        imu_raw = Imu()  # Raw IMU data
        mag_msg = MagneticField()  # Magnetometer data

        if self.bno.begin() is not True:
            print("Error initializing device")
            exit()
        time.sleep(1)
        self.bno.setExternalCrystalUse(True)
        r = self.create_rate(100)

        while self.is_loop:
            accelerometer = self.bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            magnetometer = self.bno.getVector(BNO055.VECTOR_MAGNETOMETER)
            gyroscope = self.bno.getVector(BNO055.VECTOR_GYROSCOPE)
            euler = self.bno.getVector(BNO055.VECTOR_EULER)
            orientation = self.bno.getQuat()
            linearaccel = self.bno.getVector(BNO055.VECTOR_LINEARACCEL)
            grabvity = self.bno.getVector(BNO055.VECTOR_GRAVITY)

            imu_raw = Imu()

            imu_raw.header.stamp = rospy.Time.now()
            imu_raw.header.frame_id = frame_id
            imu_raw.header.seq = seq

            imu_raw.orientation_covariance[0] = -1
            imu_raw.linear_acceleration.x = accelerometer[0] / acc_fact
            imu_raw.linear_acceleration.y = accelerometer[1] / acc_fact
            imu_raw.linear_acceleration.z = accelerometer[2] / acc_fact
            imu_raw.linear_acceleration_covariance[0] = -1
            imu_raw.angular_velocity.x = gyroscope[0] / gyr_fact
            imu_raw.angular_velocity.y = gyroscope[1] / gyr_fact
            imu_raw.angular_velocity.z = gyroscope[2] / gyr_fact
            imu_raw.angular_velocity_covariance[0] = -1
            self.pub_raw.publish(imu_raw)

            # Publish filtered data
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = frame_id
            imu_data.header.seq = seq
            imu_data.orientation.w = orientation[0]
            imu_data.orientation.x = orientation[1]
            imu_data.orientation.y = orientation[2]
            imu_data.orientation.z = orientation[3]
            imu_data.orientation_covariance[0] = -1

            imu_data.linear_acceleration.x = linearaccel[0] / acc_fact
            imu_data.linear_acceleration.y = linearaccel[1] / acc_fact
            imu_data.linear_acceleration.z = linearaccel[2] / acc_fact
            imu_data.linear_acceleration_covariance[0] = -1
            imu_data.angular_velocity.x = gyroscope[0] / gyr_fact
            imu_data.angular_velocity.y = gyroscope[1] / gyr_fact
            imu_data.angular_velocity.z = gyroscope[2] / gyr_fact
            imu_data.angular_velocity_covariance[0] = -1
            self.pub_data.publish(imu_data)

            # Publish magnetometer data
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.header.frame_id = frame_id
            mag_msg.header.seq = seq
            mag_msg.magnetic_field.x = magnetometer[0] / mag_fact
            mag_msg.magnetic_field.y = magnetometer[1] / mag_fact
            mag_msg.magnetic_field.z = magnetometer[2] / mag_fact
            self.pub_mag.publish(mag_msg)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "mobile_imu_state"
            t.child_frame_id = "mobile_imu_base"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = imu_data.orientation.x
            t.transform.rotation.y = imu_data.orientation.y
            t.transform.rotation.z = imu_data.orientation.z
            t.transform.rotation.w = imu_data.orientation.w

            seq += 1

            r.sleep()

    def stop(self):
        self.is_loop = False


def main(args=None):
    rclpy.init(args=args)

    manual = Manual()

    rclpy.spin(manual)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manual.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
