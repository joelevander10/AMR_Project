import time
import board
import busio
import adafruit_bno055
import math
import rospy
from sensors.sensor_data import SensorData

# Initialize the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Check if the sensor is connected to the I2C bus
if not is_sensor_connected_to_i2c(i2c):
    print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
    exit(1)
print("BNO055 sensor detected on the I2C bus")

# Initialize the BNO055 sensor
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Initialize the sensor
try:
    sensor.mode = adafruit_bno055.NDOF_MODE
except (RuntimeError, OSError) as e:
    print("Failed to initialize BNO055 sensor: {}".format(e))
    exit(1)
print("BNO055 sensor initialized")

# Get the initial orientation
initial_euler = sensor.euler
initial_pitch, initial_roll, initial_yaw = initial_euler[1], initial_euler[2], initial_euler[0]
print("Initial orientation:")
print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(initial_pitch, initial_roll, initial_yaw))

def imu_data_collection():
    sensor_data = SensorData()
    
    # Read acceleration (unit: m/s^2)
    acceleration = sensor.acceleration
    sensor_data.acceleration = acceleration
    
    # Read magnetic field strength (unit: microtesla)
    magnetic = sensor.magnetic
    sensor_data.magnetic = magnetic
    
    # Read gyroscope data (unit: degrees/sec)
    gyro = sensor.gyro
    sensor_data.gyroscope = gyro
    
    # Read linear acceleration (unit: m/s^2)
    linear_acceleration = sensor.linear_acceleration
    sensor_data.linear_acceleration = linear_acceleration
    
    # Read gravity vector (unit: m/s^2)
    gravity = sensor.gravity
    sensor_data.gravity = gravity
    
    # Read Euler angles (unit: degrees)
    euler = sensor.euler
    sensor_data.euler = euler
    
    # Read quaternion (no unit)
    quaternion = sensor.quaternion
    sensor_data.quaternion = quaternion
    
    # Calculate the relative orientation
    relative_pitch = euler[1] - initial_pitch
    relative_roll = euler[2] - initial_roll
    relative_yaw = euler[0] - initial_yaw
    sensor_data.relative_orientation = (relative_pitch, relative_roll, relative_yaw)
    
    return sensor_data

def is_sensor_connected_to_i2c(i2c):
    try:
        while not i2c.try_lock():
            pass
        i2c.unlock()
        return True
    except Exception:
        return False
