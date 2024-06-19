import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import math

min_time_threshold = 0.01
velocity_threshold = 0.23
moving_average_window = 2

previous_centroid = None
previous_time = None
velocity_history = []
data_count = 0
averaged_data_count = 0

def lidar_callback(data):
    global previous_centroid, previous_time, velocity_history, data_count, averaged_data_count
    
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
    
    if is_lidar_stationary(points):
        velocity_history = []
        previous_centroid = None
        previous_time = None
        rospy.loginfo("LiDAR is stationary.")
        return
    
    current_centroid = centroid(points)
    current_time = rospy.Time.now().to_sec()
    print(f"Centroid: x={current_centroid[0]:.6f}, y={current_centroid[1]:.6f}, z={current_centroid[2]:.6f}")
    
    if previous_centroid is not None:
        dist = distance(current_centroid, previous_centroid)
        time_diff = current_time - previous_time
        
        if time_diff > min_time_threshold:
            velocity = dist / time_diff
            
            if velocity < velocity_threshold:
                velocity = 0.0
            
            velocity_history.append(velocity)
            
            if len(velocity_history) == moving_average_window:
                avg_velocity = moving_average(velocity_history)
                print(f"Velocity using Centroid: {avg_velocity:.6f} m/s")
                print("---")
                velocity_history = []
                averaged_data_count += 1
        else:
            rospy.logwarn("Time difference too small. Skipping velocity calculation.")
    
    previous_centroid = current_centroid
    previous_time = current_time
    
    data_count += 1

def lidar_data_collection():
    rospy.Subscriber('/lslidar_point_cloud', PointCloud2, lidar_callback)
    rospy.spin()

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
