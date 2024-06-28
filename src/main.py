import threading
import rospy
from sensors import lidar
from sensors import imu
from sensors.sensor_data import SensorData
from sensor_msgs.msg import PointCloud2
import math
import time
import signal
import time
import board
import busio
import adafruit_bno055
import matplotlib
matplotlib.use('Agg')  # Set the backend to 'Agg' for non-interactive plotting
import matplotlib.pyplot as plt
import signal
import json
import os
import numpy as np
import scipy
import scipy.signal

def print_sensor_data(sensor_data):
    print("Acceleration:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.acceleration[0], sensor_data.acceleration[1], sensor_data.acceleration[2],
        math.sqrt(sensor_data.acceleration[0]**2 + sensor_data.acceleration[1]**2 + sensor_data.acceleration[2]**2)
    ))
    
    print("Gyroscope:")
    print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(
        sensor_data.gyroscope[0], sensor_data.gyroscope[1], sensor_data.gyroscope[2],
        math.sqrt(sensor_data.gyroscope[0]**2 + sensor_data.gyroscope[1]**2 + sensor_data.gyroscope[2]**2)
    ))
    
    print("Centroid: x={:.6f}, y={:.6f}, z={:.6f}".format(
        sensor_data.centroid[0], sensor_data.centroid[1], sensor_data.centroid[2]
    ))
    
    if sensor_data.velocity is not None:
        print("Velocity using Centroid: {:.6f} m/s".format(sensor_data.velocity))
    else:
        print("Velocity using Centroid: N/A")
    
    print("======================================")

acceleration_data = []
integrated_acceleration_data = []
summed_velocity_data = []
timestamps = []
    
def signal_handler(sig, frame):
    print("\nCtrl+C ditekan. Menyimpan plot dan keluar...")
    imu.plot_imu_data(timestamps, acceleration_data, integrated_acceleration_data, summed_velocity_data)
    exit(0)
    
def main():
    global acceleration_data, integrated_acceleration_data, summed_velocity_data, timestamps
    rospy.init_node('lidar_imu_listener', anonymous=True)
    
    rate = rospy.Rate(22)  # Sampling rate of 22 Hz
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    sensor_data = SensorData()
        
    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize previous velocity and time
    prev_velocity = (0.0, 0.0, 0.0)
    prev_time = time.time()
    prev_acceleration = (0.0, 0.0, 0.0)  # Initialize previous acceleration as (0, 0, 0)
    
    # Initialize variables for tracking no motion
    last_stop_time = time.time()
    velocity_forced_zero = False

    # Calibration
    print("Kalibrasi selama 10 detik...")
    calibration_start_time = time.time()
    calibration_data = []

    while time.time() - calibration_start_time < 10.0:  # Calibrate for 10 seconds
        acceleration = sensor.linear_acceleration
        calibration_data.append(acceleration)
        time.sleep(1/115)  # Maintain the sampling rate at 64 Hz
    
    # Calculate average calibration values
    avg_calibration = tuple(sum(axis) / len(axis) for axis in zip(*calibration_data))
    print(f"Nilai kalibrasi: {avg_calibration}")

    print("Pengumpulan data dimulai. Pengumpulan data akan berhenti setelah 1 detik.")
    start_time = time.time()  # Record the start time
    
    try:
        while not rospy.is_shutdown():
            sensor_data, velocity, prev_time, prev_acceleration = imu.imu_data_collection(prev_time, prev_acceleration)
        
            # Subtract calibration values from acceleration
            calibrated_acceleration = tuple(accel - cal for accel, cal in zip(sensor_data.acceleration, avg_calibration))
            
            # Apply Kalman Filter
            filtered_acceleration = imu.kalman_filter(calibrated_acceleration, Q=0.001, R=0.1)

            print(f"Acceleration: {filtered_acceleration}")
            
            # Menentukan batas ambang akselerasi untuk dianggap tidak ada pergerakan
            acceleration_threshold = 0.1  # Sesuaikan sesuai kebutuhan

            # Menentukan durasi (dalam detik) akselerasi harus di bawah ambang batas untuk dianggap tidak ada pergerakan
            no_motion_duration = 0.5  # Sesuaikan sesuai kebutuhan

            # Periksa apakah akselerasi mendekati 0 untuk semua sumbu
            if all(abs(accel) <= acceleration_threshold for accel in filtered_acceleration):
                if time.time() - last_stop_time >= no_motion_duration and not velocity_forced_zero:
                    velocity = (0.0, 0.0, 0.0)
                    velocity_forced_zero = True
                    print("Velocity dipaksa menjadi 0 karena tidak ada pergerakan.")
            else:
                last_stop_time = time.time()
                velocity_forced_zero = False
            
            print(f"Velocity: {velocity}")
            # Store data for plotting
            acceleration_data.append(filtered_acceleration)
            # Reset velocity to zero at the start of each integration
            if len(integrated_acceleration_data) == 0:
                velocity = (0.0, 0.0, 0.0)
            
            integrated_acceleration_data.append(velocity)
            summ_velocity = tuple(prev_vel + vel for prev_vel, vel in zip(prev_velocity, velocity))
            
            if velocity_forced_zero:
                velocity = (0.0, 0.0, 0.0)
                summ_velocity = (0.0, 0.0, 0.0)
            
            print(f"Sum Velocity: {summ_velocity}")
            print(f"-----------------------------------\n")
            summed_velocity_data.append(summ_velocity)
            timestamps.append(prev_time - start_time)
            
            prev_velocity = summed_velocity_data[-1]
            
            time.sleep(1/115)  # Sleep for 1/115 seconds to achieve 64 Hz sampling rate
        
            lidar_data = rospy.wait_for_message('/lslidar_point_cloud', PointCloud2)
            lidar.lidar_callback(lidar_data, sensor_data)
            
            print_sensor_data(sensor_data)
            
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    window_size = min(2, len(acceleration_data) // 2)  # Use at least 2 points, but no more than half the data
    if window_size > 0:
        acceleration_data = imu.moving_average(acceleration_data, window_size)
        integrated_acceleration_data = imu.moving_average(integrated_acceleration_data, window_size)
        summed_velocity_data = imu.moving_average(summed_velocity_data, window_size)
        timestamps = imu.moving_average(timestamps, window_size)

    imu.plot_imu_data(timestamps, acceleration_data, integrated_acceleration_data, summed_velocity_data)

if __name__ == '__main__':
    main()
