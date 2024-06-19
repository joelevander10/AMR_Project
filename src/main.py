import threading
from sensors import lidar
from sensors import imu

def main():
    lidar_thread = threading.Thread(target=lidar.start_lidar_subscriber)
    imu_thread = threading.Thread(target=imu.imu_data_collection)
    
    lidar_thread.start()
    imu_thread.start()
    
    lidar_thread.join()
    imu_thread.join()

if __name__ == '__main__':
    main()
