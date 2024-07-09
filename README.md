# AMR With Jetson Nano

# 1. Instalasi ROS
## Configure your Ubuntu repositories
Konfigurasikan repositori Ubuntu Anda untuk mengizinkan "terbatas", "semesta", dan "multiverse". Anda dapat mengikuti panduan Ubuntu untuk petunjuk melakukan hal ini.

## Setup your sources.list
- sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Set up your keys
- sudo apt install curl # if you haven't already installed curl
- curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

## Installation
- sudo apt update
- sudo apt install ros-noetic-desktop-full

## Environment setup
- source /opt/ros/noetic/setup.bash
- echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
- source ~/.bashrc

## Dependencies for building packages
- sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
- sudo apt install python3-rosdep
- sudo rosdep init
- rosdep update

# 2. Software Operation Example
## Membangun ruang kerja dan membangun lingkungan kompilasi
- mkdir -p ~/catkin_ws/src
- cd ~/catkin_ws/src
- catkin_create_pkg lidar_imu_ros rospy sensor_msgs geometry_msgs
- cd ~/catkin_ws/src/lidar_imu_ros
- mkdir scripts

## Download ROS Driver
- git clone -b C32_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git
- git clone https://github.com/dheera/ros-imu-bno055.git
- cd /home/amr-project/roskin_ws/src/ros-imu-bno055
- mv CMakeLists.ros2.txt CMakeLists.txt

## Tambahkan dependensi yang diperlukan di file CMakeLists.txt di dalam package lidar_imu_ros.
Tambahkan baris berikut di file CMakeLists.txt:
```
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs
)
```

## Compile and package
- cd ~/catkin_ws
- catkin_make
- cd ~/catkin_ws/src/lidar_imu_ros/scripts
- chmod +x imu_node.py lidar_imu_listener.py

## Jalankan Program
- source devel/setup.bash
- sudo ip link set eth0 up
- sudo ip addr add 192.168.1.102 dev eth0
- roslaunch lslidar_cx_driver lslidar_cx.launch
- rosrun lidar_imu_ros imu_node.py
- mkdir lidar.py
```
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

def lidar_callback(data, sensor_data):
    global previous_centroid, previous_time, velocity_history
    
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
    
    if is_lidar_stationary(points):
        velocity_history = []
        previous_centroid = None
        previous_time = None
        rospy.loginfo("LiDAR is stationary.")
        sensor_data.velocity = 0.0  # Set default velocity to 0.0
        return
    
    current_centroid = centroid(points)
    current_time = rospy.Time.now().to_sec()
    
    sensor_data.centroid = current_centroid
    sensor_data.velocity = 0.0  # Set default velocity to 0.0
    
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
```

- mkdir imu_node.py
```
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055

def imu_publisher():
    rospy.init_node('imu_node', anonymous=True)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(100)  # Publikasikan data IMU dengan frekuensi 100 Hz

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'

        acceleration = sensor.linear_acceleration
        imu_msg.linear_acceleration.x = acceleration[0]
        imu_msg.linear_acceleration.y = acceleration[1]
        imu_msg.linear_acceleration.z = acceleration[2]

        gyroscope = sensor.gyro
        imu_msg.angular_velocity.x = gyroscope[0]
        imu_msg.angular_velocity.y = gyroscope[1]
        imu_msg.angular_velocity.z = gyroscope[2]

        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
```

- mkdir sensor_data.py
```
class SensorData:
    def __init__(self):
        self.acceleration = None
        self.magnetic = None
        self.gyroscope = None
        self.linear_acceleration = None
        self.gravity = None
        self.euler = None
        self.quaternion = None
        self.relative_orientation = None
        self.centroid = None
        self.velocity = None
```

- mkdir lidar_imu_listener.py
```
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
from sensor_data import SensorData
import lidar
import math
import time

def lidar_callback(data, sensor_data):
    lidar.lidar_callback(data, sensor_data)

def imu_callback(data, sensor_data):
    sensor_data.acceleration = (data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
    sensor_data.gyroscope = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

def main():
    rospy.init_node('lidar_imu_listener', anonymous=True)

    sensor_data = SensorData()

    lidar_sub = rospy.Subscriber('/lslidar_point_cloud', PointCloud2, lidar_callback, sensor_data)
    imu_sub = rospy.Subscriber('imu/data', Imu, imu_callback, sensor_data)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # Lakukan pemrosesan data dengan frekuensi 10 Hz

    # Initialize previous velocity and time for IMU velocity calculation
    prev_velocity = (0.0, 0.0, 0.0)
    prev_time = time.time()
    prev_acceleration = (0.0, 0.0, 0.0)

    while not rospy.is_shutdown():
        twist_msg = Twist()

        # Calculate velocity from LiDAR data using the lidar module
        lidar_velocity = sensor_data.velocity

        # Calculate velocity from IMU acceleration data
        current_time = time.time()
        dt = current_time - prev_time
        acceleration = sensor_data.acceleration

        if acceleration is not None and prev_velocity is not None:
            imu_velocity = tuple((prev_vel + accel) * dt / 2 for prev_vel, accel in zip(prev_velocity, acceleration))
        else:
            imu_velocity = (0.0, 0.0, 0.0)  # Set default velocity if data is not available

        # Print velocity from LiDAR and IMU
        if lidar_velocity is not None:
            rospy.loginfo("LiDAR Velocity: {:.3f} m/s".format(lidar_velocity))
        else:
            rospy.loginfo("LiDAR Velocity: N/A")

        rospy.loginfo("IMU Velocity: X: {:.3f} Y: {:.3f} Z: {:.3f} m/s".format(imu_velocity[0], imu_velocity[1], imu_velocity[2]))

        # Update previous velocity, time, and acceleration for next iteration
        prev_velocity = imu_velocity
        prev_time = current_time
        prev_acceleration = acceleration

        # Set linear velocity based on LiDAR data
        if lidar_velocity is not None:
            twist_msg.linear.x = lidar_velocity
        else:
            twist_msg.linear.x = 0.0  # Set default linear velocity if LiDAR data is not available

        # Set angular velocity based on IMU gyroscope data
        if sensor_data.gyroscope is not None:
            twist_msg.angular.z = sensor_data.gyroscope[2]

        twist_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
```


- rosrun lidar_imu_ros lidar_imu_listener.py
  
# 3. Mengambil data 3D Lidar dari ROS ke Python
- source /opt/ros/noetic/setup.bash
- Untuk mengambil data xyz dari ROS, maka bisa gunakan kode python ROSLidar.py

# 4. Cartographer SLAM
- sudo apt-get update
- sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
- mkdir catkin_ws
- cd catkin_ws
- wstool init src
- wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
- wstool update -t src
- sudo rosdep init
- rosdep update
- rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
- src/cartographer/scripts/install_abseil.sh
- sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
- catkin_make_isolated --install --use-ninja
- 
