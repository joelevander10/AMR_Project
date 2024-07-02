# LSLidar-C16-ROS for Linux

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

## Download ROS Driver
- git clone -b C32_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git
- git clone https://github.com/dheera/ros-imu-bno055.git
- cd /home/amr-project/roskin_ws/src/ros-imu-bno055
- mv CMakeLists.ros2.txt CMakeLists.txt
  
## Compile and package
- cd ~/catkin_ws
- catkin_make

## Jalankan Program
- source devel/setup.bash
- sudo ip link set eth0 up
- sudo ip addr add 192.168.1.102 dev eth0
- roslaunch lslidar_cx_driver lslidar_cx.launch
- roslaunch imu_bno055 imu.launch
  
# 3. Mengambil data 3D Lidar dari ROS ke Python
- source /opt/ros/noetic/setup.bash
- Untuk mengambil data xyz dari ROS, maka bisa gunakan kode python ROSLidar.py
