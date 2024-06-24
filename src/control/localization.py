import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.common import Q_discrete_white_noise

class Localization:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/amr/pose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/amr/odom', Odometry, queue_size=10)
        
        self.uwb_sub = rospy.Subscriber('/uwb/range', Range, self.uwb_callback)
        
        self.dt = 1.0 / 22.0  # Assuming 22 Hz sampling rate
        self.ukf = self.initialize_ukf()
        
        self.uwb_ranges = []
        self.uwb_positions = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])  # Example UWB anchor positions
    
    def initialize_ukf(self):
        ukf = UnscentedKalmanFilter(dim_x=5, dim_z=3, dt=self.dt, fx=self.process_model, hx=self.measurement_model)
        ukf.x = np.array([0, 0, 0, 0, 0])  # Initial state [x, y, yaw, v, w]
        ukf.P = np.diag([1, 1, 0.1, 0.1, 0.1])  # Initial covariance
        ukf.R = np.diag([0.1, 0.1, 0.05])  # Measurement noise covariance
        ukf.Q = Q_discrete_white_noise(dim=5, dt=self.dt, var=0.01)  # Process noise covariance
        return ukf
    
    def process_model(self, x, dt):
        # State: [x, y, yaw, v, w]
        x_pos = x[0] + x[3] * np.cos(x[2]) * dt
        y_pos = x[1] + x[3] * np.sin(x[2]) * dt
        yaw = x[2] + x[4] * dt
        v = x[3]
        w = x[4]
        return np.array([x_pos, y_pos, yaw, v, w])
    
    def measurement_model(self, x):
        # Measurement: [x, y, yaw]
        return x[:3]
    
    def uwb_callback(self, msg):
        self.uwb_ranges.append(msg.range)
        if len(self.uwb_ranges) == 4:
            self.uwb_ranges = []
            self.ukf.update(self.trilaterate())
    
    def trilaterate(self):
        A = 2 * (self.uwb_positions[1:] - self.uwb_positions[0])
        b = np.sum(self.uwb_positions[1:]**2 - self.uwb_positions[0]**2, axis=1) - (np.array(self.uwb_ranges[1:])**2 - self.uwb_ranges[0]**2)
        pos = np.linalg.lstsq(A, b, rcond=None)[0]
        return np.array([pos[0], pos[1], 0])
    
    def update(self, sensor_data):
        # Predict
        self.ukf.predict()
        
        # Update with IMU data
        quaternion = sensor_data.quaternion
        euler = Rotation.from_quat(quaternion).as_euler('xyz', degrees=False)
        yaw = euler[2]
        self.ukf.update(np.array([0, 0, yaw]))
        
        # Update with LiDAR data
        if sensor_data.velocity is not None:
            self.ukf.update(np.array([0, 0, 0, sensor_data.velocity, 0]))
        
        # Publish estimated pose and odometry
        self.publish_pose()
        self.publish_odom()
    
    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.ukf.x[0]
        pose_msg.pose.pose.position.y = self.ukf.x[1]
        pose_msg.pose.pose.orientation = Rotation.from_euler('z', self.ukf.x[2]).as_quat()
        pose_msg.pose.covariance = self.ukf.P[:3, :3].flatten().tolist()
        self.pose_pub.publish(pose_msg)
    
    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.ukf.x[0]
        odom_msg.pose.pose.position.y = self.ukf.x[1]
        odom_msg.pose.pose.orientation = Rotation.from_euler('z', self.ukf.x[2]).as_quat()
        odom_msg.twist.twist.linear.x = self.ukf.x[3]
        odom_msg.twist.twist.angular.z = self.ukf.x[4]
        self.odom_pub.publish(odom_msg)
