import rospy
from sensors import lidar
from sensors import imu

def main():
    rospy.init_node('lidar_imu_listener', anonymous=True)
    
    rate = rospy.Rate(22)  # Sampling rate of 22 Hz
    
    while not rospy.is_shutdown():
        lidar.lidar_data_collection()
        imu.imu_data_collection()
        
        rate.sleep()

if __name__ == '__main__':
    main()
