import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import math
from sensors import lidar
from sensors import imu
from sensors.sensor_data import SensorData
from sensor_msgs.msg import PointCloud2
import numpy as np
from pykalman import KalmanFilter

class AMRLocalization:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/lslidar_point_cloud', PointCloud2, self.lidar_callback)
        self.imu_data = None
        self.lidar_data = None
        self.kalman_filter = self.initialize_kalman_filter()

    def initialize_kalman_filter(self):
        # Initialize the Kalman filter with appropriate initial state and covariance matrices
        initial_state_mean = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        initial_state_covariance = np.eye(6)
        transition_matrices = np.eye(6)
        observation_matrices = np.eye(6)
        kf = KalmanFilter(transition_matrices=transition_matrices,
                          observation_matrices=observation_matrices,
                          initial_state_mean=initial_state_mean,
                          initial_state_covariance=initial_state_covariance)
        return kf

    def lidar_callback(self, data):
        self.lidar_data = data

    def imu_data_collection(self):
        self.imu_data = imu.imu_data_collection()

    def update_localization(self):
        if self.imu_data is None or self.lidar_data is None:
            return

        # Preprocess IMU data
        imu_acc = np.array([self.imu_data.acceleration[0], self.imu_data.acceleration[1], self.imu_data.acceleration[2]])
        imu_gyro = np.array([self.imu_data.gyroscope[0], self.imu_data.gyroscope[1], self.imu_data.gyroscope[2]])
        imu_quat = np.array([self.imu_data.quaternion[0], self.imu_data.quaternion[1], self.imu_data.quaternion[2], self.imu_data.quaternion[3]])

        # Preprocess LiDAR data
        lidar_points = list(pc2.read_points(self.lidar_data, field_names=("x", "y", "z"), skip_nans=True))
        lidar_centroid = lidar.centroid(lidar_points)

        # Perform EKF update
        observations = np.concatenate((imu_acc, imu_gyro, lidar_centroid))
        self.kalman_filter.em(observations, n_iter=5)
        filtered_state_mean, filtered_state_covariance = self.kalman_filter.filter(observations)

        # Extract the updated pose estimate from the filtered state
        pose_estimate = filtered_state_mean[:6]  # [x, y, z, roll, pitch, yaw]

        return pose_estimate

def main():
    rospy.init_node('amr_localization', anonymous=True)

    amr_localization = AMRLocalization()

    rate = rospy.Rate(10)  # Update rate of 10 Hz

    while not rospy.is_shutdown():
        amr_localization.imu_data_collection()
        pose_estimate = amr_localization.update_localization()

        if pose_estimate is not None:
            print("Estimated Pose:")
            print("Position: x={:.3f}, y={:.3f}, z={:.3f}".format(pose_estimate[0], pose_estimate[1], pose_estimate[2]))
            print("Orientation: roll={:.3f}, pitch={:.3f}, yaw={:.3f}".format(pose_estimate[3], pose_estimate[4], pose_estimate[5]))
            print("======================================")

        rate.sleep()

if __name__ == '__main__':
    main()
