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
        
        self.lidar_sub = rospy.Subscriber('/lslidar_point_cloud', PointCloud2, self.lidar_callback)
        self.pose_pub = rospy.Publisher('/amr_pose', PoseStamped, queue_size=1)
        
        self.tf_listener = tf.TransformListener()
    
    def lidar_callback(self, data):
        # Transformasikan titik PointCloud2 dari frame LiDAR ke frame robot
        try:
            point_cloud = np.array(list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
            point_cloud_robot = self.transform_pointcloud(point_cloud, 'base_link', data.header.frame_id)
        except tf.Exception as e:
            rospy.logerr("Error transforming PointCloud2: %s" % str(e))
            return
        
        # Perbarui peta grid berdasarkan pengukuran LiDAR
        self.update_map(point_cloud_robot)
        
        # Perkirakan pose robot menggunakan data IMU
        pose_estimate = self.estimate_pose()
        
        # Gabungkan perkiraan pose dengan pembaruan peta grid
        self.update_map_with_pose(pose_estimate)
        
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
    
    def update_map(self, point_cloud):
        # Perbarui nilai probabilitas pada peta grid berdasarkan pengukuran LiDAR
        # Implementasikan algoritma seperti Bayes Filter atau Particle Filter di sini
        pass
    
    def estimate_pose(self):
        # Perkirakan pose robot menggunakan data IMU
        # Gunakan teknik seperti Odometry untuk memperkirakan pergerakan robot
        pass
    
    def update_map_with_pose(self, pose_estimate):
        # Perbarui nilai probabilitas pada peta grid berdasarkan perkiraan pose dari IMU
        pass
    
    def get_robot_pose(self):
        # Tentukan pose robot yang paling mungkin berdasarkan peta grid yang diperbarui
        pass
    
    def publish_pose(self, robot_pose):
        # Publikasikan pose robot
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = robot_pose
        self.pose_pub.publish(pose_msg)
