mport rospy
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055

def imu_publisher():
    rospy.init_node('imu_node', anonymous=True)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(22)  # Publikasikan data IMU dengan frekuensi 100 Hz

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
