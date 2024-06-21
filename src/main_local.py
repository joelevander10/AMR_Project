import rospy
from sensors import lidar
from sensors import imu
from sensors.sensor_data import SensorData
from sensor_msgs.msg import PointCloud2
from control.grid_localization import GridLocalization

def main():
    rospy.init_node('lidar_imu_localization', anonymous=True)
    
    rate = rospy.Rate(22)  # Sampling rate of 22 Hz
    
    localization = GridLocalization()
    
    while not rospy.is_shutdown():
        sensor_data = SensorData()
        
        imu.imu_data_collection(sensor_data)
        
        lidar_data = rospy.wait_for_message('/lslidar_point_cloud', PointCloud2)
        lidar.lidar_callback(lidar_data, sensor_data)
        
        localization.update(lidar_data, sensor_data)
        
        rate.sleep()

if __name__ == '__main__':
    main()
