import threading
import rospy
from sensors import lidar
from sensors import imu

def main():
    rospy.init_node('lidar_imu_listener', anonymous=True)
    
    lidar_thread = threading.Thread(target=lidar.start_lidar_subscriber)
    imu_thread = threading.Thread(target=imu.imu_data_collection)
    
    lidar_thread.start()
    imu_thread.start()
    
    rospy.spin()
    
    lidar_thread.join()
    imu_thread.join()

if __name__ == '__main__':
    main()
