import time
import board
import busio
import adafruit_bno055
import math

def imu_data_collection():
    # Initialize the I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Check if the sensor is connected to the I2C bus
    if not is_sensor_connected_to_i2c(i2c):
        print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
        return
    print("BNO055 sensor detected on the I2C bus")
    
    # Initialize the BNO055 sensor
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    # Initialize the sensor
    try:
        sensor.mode = adafruit_bno055.NDOF_MODE
    except (RuntimeError, OSError) as e:
        print("Failed to initialize BNO055 sensor: {}".format(e))
        return
    print("BNO055 sensor initialized")
    
    # Get the initial orientation
    initial_euler = sensor.euler
    initial_pitch, initial_roll, initial_yaw = initial_euler[1], initial_euler[2], initial_euler[0]
    print("Initial orientation:")
    print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(initial_pitch, initial_roll, initial_yaw))
    
    # Main loop
    while True:
        start_time = time.time()
        
        # Read acceleration (unit: m/s^2)
        acceleration = sensor.acceleration
        accel_magnitude = math.sqrt(acceleration[0]**2 + acceleration[1]**2 + acceleration[2]**2)
        print("Acceleration:")
        print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(acceleration[0], acceleration[1], acceleration[2], accel_magnitude))
        
        # Read magnetic field strength (unit: microtesla)
        magnetic = sensor.magnetic
        mag_magnitude = math.sqrt(magnetic[0]**2 + magnetic[1]**2 + magnetic[2]**2)
        print("Magnetic Field Strength:")
        print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(magnetic[0], magnetic[1], magnetic[2], mag_magnitude))
        
        # Read gyroscope data (unit: degrees/sec)
        gyro = sensor.gyro
        gyro_magnitude = math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)
        print("Gyroscope:")
        print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(gyro[0], gyro[1], gyro[2], gyro_magnitude))
        
        # Read linear acceleration (unit: m/s^2)
        linear_acceleration = sensor.linear_acceleration
        linear_accel_magnitude = math.sqrt(linear_acceleration[0]**2 + linear_acceleration[1]**2 + linear_acceleration[2]**2)
        print("Linear Acceleration:")
        print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2], linear_accel_magnitude))
        
        # Read gravity vector (unit: m/s^2)
        gravity = sensor.gravity
        gravity_magnitude = math.sqrt(gravity[0]**2 + gravity[1]**2 + gravity[2]**2)
        print("Gravity Vector:")
        print("X: {:.3f} Y: {:.3f} Z: {:.3f} Magnitude: {:.3f}".format(gravity[0], gravity[1], gravity[2], gravity_magnitude))
        
        # Read Euler angles (unit: degrees)
        euler = sensor.euler
        pitch, roll, yaw = euler[1], euler[2], euler[0]
        print("Euler Angles:")
        print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(pitch, roll, yaw))
        
        # Read quaternion (no unit)
        quaternion = sensor.quaternion
        print("Quaternion:")
        print("W: {:.3f} X: {:.3f} Y: {:.3f} Z: {:.3f}".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        
        # Calculate the relative orientation
        relative_pitch = pitch - initial_pitch
        relative_roll = roll - initial_roll
        relative_yaw = yaw - initial_yaw
        print("Relative Orientation:")
        print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(relative_pitch, relative_roll, relative_yaw))
        
        print("======================================")
        
        dt = time.time() - start_time
        sampling_rate = 1 / dt
        print("Sampling Rate: {:.2f} Hz".format(sampling_rate))
        
        time.sleep(0.1)

def is_sensor_connected_to_i2c(i2c):
    try:
        while not i2c.try_lock():
            pass
        i2c.unlock()
        return True
    except Exception:
        return False
