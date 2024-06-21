import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf

class GridLocalization:
    def __init__(self):
        self.map_resolution = 0.05  # Resolusi peta dalam meter
        self.map_size = (200, 200)  # Ukuran peta dalam sel
        self.map = np.zeros(self.map_size, dtype=np.float32)  # Inisialisasi peta dengan nilai 0
        
        self.pose_pub = rospy.Publisher('/amr_pose', PoseStamped, queue_size=1)
        
        self.tf_listener = tf.TransformListener()
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
        self.alpha1 = 0.1  # Parameter kontrol untuk update peta
        self.alpha2 = 0.1  # Parameter kontrol untuk update peta
        self.alpha3 = 0.1  # Parameter kontrol untuk update peta
        self.alpha4 = 0.1  # Parameter kontrol untuk update peta
        
        self.sigma_hit = 0.5  # Standar deviasi untuk model likelihood
        self.sigma_short = 0.1  # Standar deviasi untuk model likelihood
        self.z_hit = 0.8  # Probabilitas pengukuran akurat
        self.z_short = 0.1  # Probabilitas pengukuran terlalu pendek
        self.z_max = 0.1  # Probabilitas pengukuran tidak terdeteksi
        self.z_rand = 0.1  # Probabilitas pengukuran acak
    
    def update(self, lidar_data, sensor_data):
        # Transformasikan titik PointCloud2 dari frame LiDAR ke frame robot
        try:
            point_cloud = np.array(list(pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True)))
            point_cloud_robot = self.transform_pointcloud(point_cloud, self.base_frame, lidar_data.header.frame_id)
        except tf.Exception as e:
            rospy.logerr("Error transforming PointCloud2: %s" % str(e))
            return
        
        # Perkirakan pose robot menggunakan data IMU
        pose_estimate = self.estimate_pose(sensor_data)
        
        # Perbarui peta grid berdasarkan pengukuran LiDAR dan perkiraan pose
        self.update_map(point_cloud_robot, pose_estimate)
        
        # Tentukan pose robot yang paling mungkin
        robot_pose = self.get_robot_pose()
        
        # Publikasikan pose robot
        self.publish_pose(robot_pose)
    
    def transform_pointcloud(self, point_cloud, target_frame, source_frame):
        # Transformasikan titik PointCloud2 dari frame sumber ke frame target
        transformed_points = []
        for point in point_cloud:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = rospy.Time()
            point_stamped.point.x = point[0]
            point_stamped.point.y = point[1]
            point_stamped.point.z = point[2]
            
            try:
                transformed_point = self.tf_listener.transformPoint(target_frame, point_stamped)
                transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        return np.array(transformed_points)
    
    def update_map(self, point_cloud, pose_estimate):
        # Perbarui nilai probabilitas pada peta grid menggunakan Bayes Filter
        for i in range(self.map_size[0]):
            for j in range(self.map_size[1]):
                cell_pose = self.cell_to_pose(i, j)
                likelihood = self.calculate_likelihood(point_cloud, cell_pose, pose_estimate)
                prior = self.map[i, j]
                self.map[i, j] = self.alpha1 * prior + self.alpha2 * likelihood
        
        # Normalisasi nilai probabilitas pada peta
        self.map /= np.sum(self.map)
    
    def estimate_pose(self, sensor_data):
        # Perkirakan pose robot menggunakan data IMU dan Odometry
        quaternion = sensor_data.quaternion
        linear_acceleration = sensor_data.linear_acceleration
        angular_velocity = sensor_data.gyroscope
        
        # Hitung perubahan pose berdasarkan data IMU
        dt = 0.1  # Asumsi interval waktu 0.1 detik
        delta_position = linear_acceleration * dt**2 / 2
        delta_orientation = angular_velocity * dt
        
        # Transformasikan perubahan pose dari frame IMU ke frame robot
        delta_pose = self.transform_pose(delta_position, delta_orientation, self.base_frame, 'imu_link')
        
        # Hitung perkiraan pose robot dengan menjumlahkan perubahan pose
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            pose_estimate = PoseStamped()
            pose_estimate.header.stamp = rospy.Time.now()
            pose_estimate.header.frame_id = self.odom_frame
            pose_estimate.pose.position.x = trans[0] + delta_pose[0]
            pose_estimate.pose.position.y = trans[1] + delta_pose[1]
            pose_estimate.pose.position.z = trans[2] + delta_pose[2]
            pose_estimate.pose.orientation.x = rot[0]
            pose_estimate.pose.orientation.y = rot[1]
            pose_estimate.pose.orientation.z = rot[2]
            pose_estimate.pose.orientation.w = rot[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get robot pose from TF")
            pose_estimate = None
        
        return pose_estimate
    
    def calculate_likelihood(self, point_cloud, cell_pose, pose_estimate):
        # Hitung likelihood pengukuran LiDAR pada pose sel tertentu
        if pose_estimate is None:
            return 0.0
        
        likelihood = 0.0
        for point in point_cloud:
            # Transformasikan titik LiDAR ke frame peta
            point_map = self.transform_point(point, 'map', self.base_frame, pose_estimate)
            
            # Hitung jarak antara titik LiDAR dan pose sel
            dx = point_map[0] - cell_pose.position.x
            dy = point_map[1] - cell_pose.position.y
            dist = np.sqrt(dx**2 + dy**2)
            
            # Hitung likelihood berdasarkan model pengukuran
            p_hit = np.exp(-0.5 * (dist / self.sigma_hit)**2) / (np.sqrt(2 * np.pi) * self.sigma_hit)
            p_short = np.exp(-0.5 * (dist / self.sigma_short)**2) / (np.sqrt(2 * np.pi) * self.sigma_short)
            p_max = 1.0 if dist <= self.map_resolution else 0.0
            p_rand = 1.0 / self.map_size[0] / self.map_size[1]
            likelihood += self.z_hit * p_hit + self.z_short * p_short + self.z_max * p_max + self.z_rand * p_rand
        
        return likelihood
    
    def transform_pose(self, position, orientation, target_frame, source_frame):
        # Transformasikan pose dari frame sumber ke frame target
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = rospy.Time()
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        
        try:
            transformed_pose = self.tf_listener.transformPose(target_frame, pose_stamped)
            return (transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to transform pose")
            return (0.0, 0.0, 0.0)
    
    def transform_point(self, point, target_frame, source_frame, pose_estimate):
        # Transformasikan titik dari frame sumber ke frame target
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = rospy.Time()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        
        try:
            transformed_point = self.tf_listener.transformPoint(target_frame, point_stamped)
            return (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to transform point")
            return (0.0, 0.0, 0.0)
    
    def cell_to_pose(self, i, j):
        # Konversi indeks sel (i, j) menjadi pose dalam frame peta
        x = (i + 0.5) * self.map_resolution
        y = (j + 0.5) * self.map_resolution
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose
    
    def get_robot_pose(self):
        # Tentukan pose robot yang paling mungkin berdasarkan peta grid
        max_prob_index = np.unravel_index(np.argmax(self.map), self.map.shape)
        return self.cell_to_pose(max_prob_index[0], max_prob_index[1])
    
    def publish_pose(self, robot_pose):
        # Publikasikan pose robot
        self.pose_pub.publish(robot_pose)
