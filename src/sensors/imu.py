import time
import board
import busio
import adafruit_bno055
import math

# Initialize the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Function to check if the sensor is connected to the I2C bus
def is_sensor_connected_to_i2c():
    try:
        while not i2c.try_lock():
            pass
        i2c.unlock()
        return True
    except Exception:
        return False

# Check if the sensor is connected to the I2C bus
if not is_sensor_connected_to_i2c():
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
