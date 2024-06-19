import threading
import rospy
from sensors import lidar
from sensors import imu
from sensors.sensor_data import SensorData
from sensor_msgs.msg import PointCloud2
import math

def print_sensor_data(sensor_data):
    print("Acceleration:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.acceleration[0], sensor_data.acceleration[1], sensor_data.acceleration[2],
        math.sqrt(sensor_data.acceleration[0]**2 + sensor_data.acceleration[1]**2 + sensor_data.acceleration[2]**2)
    ))
    
    print("Magnetic Field Strength:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.magnetic[0], sensor_data.magnetic[1], sensor_data.magnetic[2],
        math.sqrt(sensor_data.magnetic[0]**2 + sensor_data.magnetic[1]**2 + sensor_data.magnetic[2]**2)
    ))
    
    print("Gyroscope:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.gyroscope[0], sensor_data.gyroscope[1], sensor_data.gyroscope[2],
        math.sqrt(sensor_data.gyroscope[0]**2 + sensor_data.gyroscope[1]**2 + sensor_data.gyroscope[2]**2)
    ))
    
    print("Linear Acceleration:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.linear_acceleration[0], sensor_data.linear_acceleration[1], sensor_data.linear_acceleration[2],
        math.sqrt(sensor_data.linear_acceleration[0]**2 + sensor_data.linear_acceleration[1]**2 + sensor_data.linear_acceleration[2]**2)
    ))
    
    print("Gravity Vector:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.gravity[0], sensor_data.gravity[1], sensor_data.gravity[2],
        math.sqrt(sensor_data.gravity[0]**2 + sensor_data.gravity[1]**2 + sensor_data.gravity[2]**2)
    ))
    
    print("Euler Angles:")
    print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(
        sensor_data.euler[1], sensor_data.euler[2], sensor_data.euler[0]
    ))
    
    print("Quaternion:")
    print("W: {:.3f} X: {:.3f} Y: {:.3f} Z: {:.3f}".format(
        sensor_data.quaternion[0], sensor_data.quaternion[1], sensor_data.quaternion[2], sensor_data.quaternion[3]
    ))
    
    print("Relative Orientation:")
    print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(
        sensor_data.relative_orientation[0], sensor_data.relative_orientation[1], sensor_data.relative_orientation[2]
    ))
    
    print("Centroid: x={:.6f}, y={:.6f}, z={:.6f}".format(
        sensor_data.centroid[0], sensor_data.centroid[1], sensor_data.centroid[2]
    ))
    
    print("Velocity using Centroid: {:.6f} m/s".format(sensor_data.velocity))
    
    print("======================================")

def main():
    rospy.init_node('lidar_imu_listener', anonymous=True)
    
    rate = rospy.Rate(22)  # Sampling rate of 22 Hz
    
    while not rospy.is_shutdown():
        sensor_data = SensorData()
        
        imu_data = imu.imu_data_collection()
        sensor_data.acceleration = imu_data.acceleration
        sensor_data.magnetic = imu_data.magnetic
        sensor_data.gyroscope = imu_data.gyroscope
        sensor_data.linear_acceleration = imu_data.linear_acceleration
        sensor_data.gravity = imu_data.gravity
        sensor_data.euler = imu_data.euler
        sensor_data.quaternion = imu_data.quaternion
        sensor_data.relative_orientation = imu_data.relative_orientation
        
        lidar_data = rospy.wait_for_message('/lslidar_point_cloud', PointCloud2)
        lidar.lidar_callback(lidar_data, sensor_data)
        
        print_sensor_data(sensor_data)
        
        rate.sleep()

if __name__ == '__main__':
    main()
