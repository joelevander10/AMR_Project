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
        self.num_particles = 1000
        self.particles = self.initialize_particles()
        self.landmarks = self.initialize_landmarks()
        self.p_hit = 0.6
        self.p_miss = 0.2
        self.p_max = 0.8
        self.p_rand = 0.1
        self.p_stay = 0.1

    def initialize_kalman_filter(self):
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
        self.update_particles()

    def imu_data_collection(self):
        self.imu_data = imu.imu_data_collection()

    def initialize_particles(self):
        particles = np.zeros((self.num_particles, 4))  # [x, y, theta, weight]
        particles[:, 0] = np.random.uniform(0, 10, self.num_particles)  # x between 0 and 10 meters
        particles[:, 1] = np.random.uniform(0, 10, self.num_particles)  # y between 0 and 10 meters
        particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.num_particles)  # theta between -pi and pi
        particles[:, 3] = 1.0 / self.num_particles  # Initialize weights equally
        return particles

    def initialize_landmarks(self):
        landmarks = np.array([[2, 2], [5, 5], [8, 3]])  # [x, y] coordinates of landmarks
        return landmarks

    def update_particles(self):
        z = self.lidar_data
        for i in range(self.num_particles):
            self.particles[i] = self.motion_model(self.particles[i])
            self.particles[i, 3] = self.measurement_model(self.particles[i], z)

        self.normalize_weights()
        self.resample_particles()

    def motion_model(self, particle):
        dt = 0.1  # Time step
        velocity = 1.0  # Constant velocity
        particle[0] += velocity * dt * np.cos(particle[2])  # Update x position
        particle[1] += velocity * dt * np.sin(particle[2])  # Update y position
        particle[2] += np.random.normal(0, 0.1)  # Add random noise to orientation
        return particle

    def measurement_model(self, particle, z):
        nearest_landmark_dist = np.min(np.linalg.norm(self.landmarks - particle[:2], axis=1))
        if nearest_landmark_dist < 1.0:  # Threshold distance for detecting a landmark
            likelihood = self.p_hit
        else:
            likelihood = self.p_miss
        return likelihood

    def normalize_weights(self):
        weights = self.particles[:, 3]
        total_weight = np.sum(weights)
        self.particles[:, 3] = weights / total_weight

    def resample_particles(self):
        weights = self.particles[:, 3]
        indices = np.random.choice(self.num_particles, size=self.num_particles, p=weights)
        self.particles = self.particles[indices, :]

    def update_localization(self):
        pose_estimate = np.mean(self.particles[:, :3], axis=0)

        if self.imu_data is not None:
            observations = np.concatenate((self.imu_data.acceleration, self.imu_data.gyroscope))
            self.kalman_filter.em(observations, n_iter=5)
            filtered_state_mean, filtered_state_covariance = self.kalman_filter.filter(observations)
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
