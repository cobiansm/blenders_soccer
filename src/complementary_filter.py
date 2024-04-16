#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import math

# Constants
alpha = 0.95  # Complementary filter constant
dt = 0.01     # Time interval between sensor readings (in seconds)

# Initial yaw estimate
yaw = 0.0
old_yaw = 0.0
drift = 0.0
betha = 2*math.pi

def complementary_filter(yaw, gyro_z, accel_yaw):
    global alpha, old_yaw, drift
    # Complementary filter equation
    yaw += alpha * (gyro_z * dt) + (1 - alpha) * accel_yaw + 0.0003069#+ 0.001745
    if -betha > yaw or yaw > betha: yaw = 0.0
    # drift = abs(yaw) - abs(old_yaw)
    # if abs(drift) <0.01:
    #     yaw += drift

    # old_yaw = yaw
    return yaw

def imu_callback(imu_msg):
    global yaw
    # Extract angular velocity around z-axis (yaw rate)
    gyro_z = imu_msg.angular_velocity.z

    # Extract orientation (yaw angle) from quaternion
    q = imu_msg.orientation
    _, _, yaw_acc = euler_from_quaternion(q.x, q.y, q.z, q.w)

    # Update yaw estimate using complementary filter
    yaw = complementary_filter(yaw, gyro_z, yaw_acc)

    # Print current yaw estimate
    rospy.loginfo("Yaw: %.5f degrees", np.degrees(yaw))

"""
def imu_callback(imu_msg):
    global yaw
    # Extract angular velocity around z-axis (yaw rate)
    gyro_z = imu_msg.angular_velocity.z

    # Extract linear acceleration along x and y axes
    accel_x = imu_msg.linear_acceleration.x
    accel_y = imu_msg.linear_acceleration.y

    # Calculate yaw angle from accelerometer readings
    accel_yaw = np.arctan2(accel_y, accel_x)

    # Update yaw estimate using complementary filter
    yaw = complementary_filter(yaw, gyro_z, accel_yaw)

    # Print current yaw estimate
    print("Yaw: %.2f degrees", np.degrees(yaw))"""

def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw).
    """
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber('robotis_1/open_cr/imu', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass
