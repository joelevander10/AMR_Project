import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import math
import random

min_time_threshold = 0.01
velocity_threshold = 0.3
moving_average_window = 5  # Meningkatkan window untuk smoothing yang lebih baik

previous_centroid = None
previous_time = None
velocity_history = []

def lidar_callback(data, sensor_data):
    global previous_centroid, previous_time, velocity_history
    
    all_points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
    
    # Ambil sampel titik-titik LiDAR
    num_samples = 100  # Jumlah titik yang akan diambil
    points = random.sample(all_points, min(num_samples, len(all_points)))
    
    # Simpan point cloud ke sensor_data
    sensor_data.point_cloud = points
    
    if is_lidar_stationary(points):
        velocity_history = []
        previous_centroid = None
        previous_time = None
        rospy.loginfo("LiDAR is stationary.")
        sensor_data.velocity = 0.0
        return
    
    current_centroid = centroid(points)
    current_time = rospy.Time.now().to_sec()
    
    sensor_data.centroid = current_centroid
    sensor_data.velocity = 0.0
    
    if previous_centroid is not None:
        dist = distance(current_centroid, previous_centroid)
        time_diff = current_time - previous_time
        
        if time_diff > min_time_threshold:
            velocity = dist / time_diff
            
            # Tingkatkan threshold kecepatan
            if velocity < velocity_threshold * 2:
                velocity = 0.0
            
            velocity_history.append(velocity)
            
            if len(velocity_history) == moving_average_window:
                avg_velocity = moving_average(velocity_history)
                sensor_data.velocity = avg_velocity
                velocity_history = []
        else:
            rospy.logwarn("Time difference too small. Skipping velocity calculation.")
    
    previous_centroid = current_centroid
    previous_time = current_time

def centroid(points):
    x_sum = y_sum = z_sum = 0
    for point in points:
        x, y, z = point
        x_sum += x
        y_sum += y
        z_sum += z
    
    num_points = len(points)
    return (x_sum / num_points, y_sum / num_points, z_sum / num_points)

def distance(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def moving_average(values):
    if len(values) == 0:
        return 0
    return sum(values) / len(values)

def is_lidar_stationary(points, threshold=0.01):
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    
    for point in points:
        x, y, z = point
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)
    
    return (max_x - min_x) < threshold and (max_y - min_y) < threshold and (max_z - min_z) < threshold
