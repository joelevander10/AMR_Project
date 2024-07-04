#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
from sensor_data import SensorData
import lidar
import csv
import time
from collections import deque
import threading
import signal
import matplotlib.pyplot as plt
import math

running = True
lidar_velocities = []
imu_velocities = []
timestamps = []

def signal_handler(sig, frame):
    global running
    print("Ctrl+C ditekan. Menghentikan program...")
    running = False

lidar_data = {}
imu_data = {}

def lidar_callback(data, sensor_data):
    lidar.lidar_callback(data, sensor_data)
    current_time = time.time() - start_time
    velocity = sensor_data.velocity
    if velocity is not None and velocity < 0.3:  # Mengabaikan kecepatan di atas 1.0 m/s
        lidar_data[current_time] = velocity

def imu_callback(data, sensor_data):
    sensor_data.acceleration = (data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
    sensor_data.gyroscope = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)


def write_data_to_csv(file_writer, data_queue):
    global running
    while running:
        if data_queue:
            data = data_queue.popleft()
            file_writer.writerow(data)
        else:
            time.sleep(0.01)  # Jeda sebentar jika tidak ada data dalam antrian

def main():
    global running, start_time
    rospy.init_node('lidar_imu_listener', anonymous=True)

    sensor_data = SensorData()

    lidar_sub = rospy.Subscriber('/lslidar_point_cloud', PointCloud2, lidar_callback, sensor_data)
    imu_sub = rospy.Subscriber('imu/data_raw', Imu, imu_callback, sensor_data)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    lidar_rate = rospy.Rate(22)  # LiDAR rate 22 Hz
    imu_rate = rospy.Rate(64)  # IMU rate 64 Hz

    # Buka file CSV untuk menyimpan data
    lidar_file = open('lidar_data.csv', 'w')
    imu_file = open('imu_data.csv', 'w')
    pointcloud_file = open('pointcloud_data.csv', 'w')

    # Buat writer CSV
    lidar_writer = csv.writer(lidar_file)
    imu_writer = csv.writer(imu_file)
    pointcloud_writer = csv.writer(pointcloud_file)

    # Tulis header untuk file CSV
    lidar_writer.writerow(['Timestamp (s)', 'Centroid X', 'Centroid Y', 'Centroid Z', 'Velocity'])
    imu_writer.writerow(['Timestamp (s)', 'Acceleration X', 'Acceleration Y', 'Acceleration Z',
                         'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z', 'Velocity X', 'Velocity Y', 'Velocity Z'])
    pointcloud_writer.writerow(['Timestamp (s)', 'X', 'Y', 'Z'])

    start_time = time.time()
    prev_velocity = (0.0, 0.0, 0.0)
    prev_time = time.time()
    prev_acceleration = (0.0, 0.0, 0.0)

    # Inisialisasi antrian data untuk setiap jenis data
    lidar_data_queue = deque()
    imu_data_queue = deque()
    pointcloud_data_queue = deque()

    # Buat thread untuk menulis data ke file CSV secara terpisah
    lidar_writer_thread = threading.Thread(target=write_data_to_csv, args=(lidar_writer, lidar_data_queue))
    imu_writer_thread = threading.Thread(target=write_data_to_csv, args=(imu_writer, imu_data_queue))
    pointcloud_writer_thread = threading.Thread(target=write_data_to_csv, args=(pointcloud_writer, pointcloud_data_queue))

    # Mulai thread
    lidar_writer_thread.start()
    imu_writer_thread.start()
    pointcloud_writer_thread.start()

    signal.signal(signal.SIGINT, signal_handler)

    last_imu_time = 0
    try:
        while running and not rospy.is_shutdown():
            current_time = time.time() - start_time  # Waktu dalam detik

            twist_msg = Twist()

            lidar_velocity = sensor_data.velocity
            lidar_centroid = sensor_data.centroid

            if lidar_velocity is not None:
                rospy.loginfo("LiDAR Velocity: {:.3f} m/s".format(lidar_velocity))
                print(f"Time: {current_time}, LiDAR Velocity: {lidar_velocity:.3f} cm/s")
            else:
                rospy.loginfo("LiDAR Velocity: N/A")
                print(f"Time: {current_time}, LiDAR Velocity: N/A")

            if lidar_centroid is not None and lidar_velocity is not None:
                lidar_data_queue.append([current_time, lidar_centroid[0], lidar_centroid[1], lidar_centroid[2], lidar_velocity])

            # Tambahkan data point cloud ke antrian
            if sensor_data.point_cloud is not None:
                for point in sensor_data.point_cloud:
                    pointcloud_data_queue.append([current_time, point[0], point[1], point[2]])

            # Set linear velocity based on LiDAR data
            twist_msg.linear.x = lidar_velocity if lidar_velocity is not None else 0.0

            lidar_rate.sleep()

            # IMU processing (64 Hz)
            for _ in range(3):  # Run IMU processing ~3 times for each LiDAR cycle
                current_time = time.time() - start_time
                if current_time - last_imu_time >= 1/64:  # Hanya catat jika sudah lewat 1/64 detik
                    dt = current_time - last_imu_time
                    acceleration = sensor_data.acceleration
                    if acceleration is not None and prev_velocity is not None:
                        imu_velocity = tuple((prev_vel + accel) * dt / 2 for prev_vel, accel in zip(prev_velocity, acceleration))
                    else:
                        imu_velocity = (0.0, 0.0, 0.0)  # Set default velocity if data is not available

                    velocity = math.sqrt(imu_velocity[0]**2 + imu_velocity[1]**2)
                    if velocity < 0.3:  # Mengabaikan kecepatan di atas 1.0 m/s
                        imu_data[current_time] = velocity
        
                    rospy.loginfo("IMU Velocity: X: {:.3f} Y: {:.3f} Z: {:.3f} m/s".format(imu_velocity[0], imu_velocity[1], imu_velocity[2]))
                    print(f"Time: {current_time}, IMU Velocity: X={imu_velocity[0]:.3f}, Y={imu_velocity[1]:.3f}, Z={imu_velocity[2]:.3f} m/s")

                    if acceleration is not None and sensor_data.gyroscope is not None:
                        imu_data_queue.append([current_time, acceleration[0], acceleration[1], acceleration[2],
                                               sensor_data.gyroscope[0], sensor_data.gyroscope[1], sensor_data.gyroscope[2],
                                               imu_velocity[0], imu_velocity[1], imu_velocity[2]])

                    # Update previous velocity, time, and acceleration for next iteration
                    prev_velocity = imu_velocity
                    prev_acceleration = acceleration
                    last_imu_time = current_time

                # Set angular velocity based on IMU gyroscope data
                if sensor_data.gyroscope is not None:
                    twist_msg.angular.z = sensor_data.gyroscope[2]

                twist_pub.publish(twist_msg)

                imu_rate.sleep()

    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna.")
    finally:
        running = False
        print("Menunggu antrian data kosong...")
        while (lidar_data_queue or imu_data_queue or pointcloud_data_queue) and running:
            time.sleep(0.1)

        print("Menunggu thread-thread selesai...")
        lidar_writer_thread.join()
        imu_writer_thread.join()
        pointcloud_writer_thread.join()
        print("Semua thread telah berhenti.")

        # Plot hasil pembacaan kecepatan LiDAR dan IMU
        plt.figure(figsize=(10, 6))
        lidar_timestamps = list(lidar_data.keys())
        lidar_velocities = list(lidar_data.values())
        print(lidar_velocities)
        imu_timestamps = list(imu_data.keys())
        imu_velocities = list(imu_data.values())
        plt.plot(lidar_timestamps, lidar_velocities, label='LiDAR Velocity')
        plt.plot(imu_timestamps, imu_velocities, label='IMU Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('LiDAR and IMU Velocity Comparison')
        plt.legend()
        plt.grid(True)
        plt.savefig('velocity_comparison.png')
        plt.close()

        # Tutup file CSV
        lidar_file.close()
        imu_file.close()
        pointcloud_file.close()

        print("Plot telah disimpan dan program selesai.")

if __name__ == '__main__':
    main()
