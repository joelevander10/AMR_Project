#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
from sensor_data import SensorData
import lidar
import math
import time

def lidar_callback(data, sensor_data):
    lidar.lidar_callback(data, sensor_data)

def imu_callback(data, sensor_data):
    sensor_data.acceleration = (data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
    sensor_data.gyroscope = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

def main():
    rospy.init_node('lidar_imu_listener', anonymous=True)

    sensor_data = SensorData()

    lidar_sub = rospy.Subscriber('/lslidar_point_cloud', PointCloud2, lidar_callback, sensor_data)
    imu_sub = rospy.Subscriber('imu/data_raw', Imu, imu_callback, sensor_data)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(32)  # Lakukan pemrosesan data dengan frekuensi 10 Hz

    # Initialize previous velocity and time for IMU velocity calculation
    prev_velocity = (0.0, 0.0, 0.0)
    prev_time = time.time()
    prev_acceleration = (0.0, 0.0, 0.0)

    while not rospy.is_shutdown():
        twist_msg = Twist()

        # Calculate velocity from LiDAR data using the lidar module
        lidar_velocity = sensor_data.velocity

        # Calculate velocity from IMU acceleration data
        current_time = time.time()
        dt = current_time - prev_time
        acceleration = sensor_data.acceleration
        
        if acceleration is not None and prev_velocity is not None:
            imu_velocity = tuple((prev_vel + accel) * dt / 2 for prev_vel, accel in zip(prev_velocity, acceleration))
        else:
            imu_velocity = (0.0, 0.0, 0.0)  # Set default velocity if data is not available

        # Print velocity from LiDAR and IMU
        # Print velocity from LiDAR and IMU
        if lidar_velocity is not None:
            rospy.loginfo("LiDAR Velocity: {:.3f} m/s".format(lidar_velocity))
        else:
            rospy.loginfo("LiDAR Velocity: N/A")
        rospy.loginfo("IMU Velocity: X: {:.3f} Y: {:.3f} Z: {:.3f} m/s".format(imu_velocity[0], imu_velocity[1], imu_velocity[2]))

        # Update previous velocity, time, and acceleration for next iteration
        prev_velocity = imu_velocity
        prev_time = current_time
        prev_acceleration = acceleration

        # Set linear velocity based on LiDAR data
        twist_msg.linear.x = lidar_velocity

        # Set angular velocity based on IMU gyroscope data
        if sensor_data.gyroscope is not None:
            twist_msg.angular.z = sensor_data.gyroscope[2]

        twist_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
