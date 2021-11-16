#!/usr/bin/env python

import rospy
import threading
import traceback
import time
import math

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
from tf.transformations import *
import tf

from bno055 import BNO055

SENSORS_GRAVITY_EARTH = 9.80665


class Manual:
    def __init__(self):
        rospy.init_node("imu_publisher", anonymous=True)
        self.imu_tf_broadcaster = tf.TransformBroadcaster()
        self.pub_data = rospy.Publisher("/mobile/imu/data", Imu, queue_size=1)
        self.pub_raw = rospy.Publisher("/mobile/imu/raw", Imu, queue_size=1)
        self.pub_mag = rospy.Publisher("/mobile/imu/mag", MagneticField, queue_size=1)
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
        r = rospy.Rate(100)
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

            current_time = rospy.Time.now()
            self.imu_tf_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                (
                    imu_data.orientation.x,
                    imu_data.orientation.y,
                    imu_data.orientation.z,
                    imu_data.orientation.w,
                ),
                current_time,
                "mobile_imu_state",
                "mobile_imu_base",
            )

            seq += 1

            r.sleep()

    def stop(self):
        self.is_loop = False


if __name__ == "__main__":
    try:
        manual = Manual()
        rospy.spin()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    finally:
        traceback.print_exc()
        manual.stop()
