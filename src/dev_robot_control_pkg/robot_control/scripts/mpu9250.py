#!/usr/bin/env python3

import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
import math
import time
import geometry_msgs.msg
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from tf2_ros import TransformBroadcaster

G = 9.80665
MagFieldConversion_uT_T = 0.000001

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

# mpu.configure()

class IMUDataNode():
    def __init__(self):
        rospy.init_node('imu_data_node', anonymous=True)

        self.imu_pub = rospy.Publisher (
             "/imu/data_raw",
             Imu,
             queue_size=10
        )

        mpu.configure()
        # time.delay(1)
        mpu.calibrate()
        # time.delay(1)
        mpu.configure()
        # time.delay(1)


        rospy.Timer(rospy.Duration(1.0/10.0), self.pub_callback)

        rospy.loginfo("IMU STARTED")

    def pub_callback(self, event):
        timestamp = rospy.Time.now()

        # create imu msg
        q0 = 1.0 #W
        q1 = 0.0 #X
        q2 = 0.0 #Y
        q3 = 0.0 #Z

        #Fill imu message
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = 'imu_frame'
        imu_msg.orientation.x = q0
        imu_msg.orientation.y = q1
        imu_msg.orientation.z = q2
        imu_msg.orientation.w = q3
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01

        gx, gy, gz = mpu.readGyroscopeMaster()
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        imu_msg.angular_velocity_covariance[0] = 0.03
        imu_msg.angular_velocity_covariance[4] = 0.03
        imu_msg.angular_velocity_covariance[8] = 0.03

        ax, ay, az = mpu.readAccelerometerMaster()
        imu_msg.linear_acceleration.x = ax*G
        imu_msg.linear_acceleration.y = ay*G
        imu_msg.linear_acceleration.z = az*G
        imu_msg.linear_acceleration_covariance[0] = 10
        imu_msg.linear_acceleration_covariance[4] = 10
        imu_msg.linear_acceleration_covariance[8] = 10

        self.imu_pub.publish(imu_msg)

def main(args=None):
    imu_data_node = IMUDataNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
