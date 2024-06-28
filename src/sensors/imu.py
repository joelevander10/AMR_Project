import time
import board
import busio
import adafruit_bno055
from sensors.sensor_data import SensorData
import matplotlib
matplotlib.use('Agg')  # Set the backend to 'Agg' for non-interactive plotting
import matplotlib.pyplot as plt
import signal
import json
import os
import numpy as np
import scipy
import scipy.signal

def is_sensor_connected_to_i2c(i2c):
    try:
        while not i2c.try_lock():
            pass
        i2c.unlock()
        return True
    except Exception:
        return False

def load_calibration(filename="bno055_calibration.json"):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            return json.load(f)
    return None

def moving_average(data, window_size):
    return [sum(data[i:i+window_size])/window_size for i in range(0, len(data), window_size)]

# Initialize the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Check if the sensor is connected to the I2C bus
if not is_sensor_connected_to_i2c(i2c):
    print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
    exit(1)
print("BNO055 sensor detected on the I2C bus")

# Initialize the BNO055 sensor
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Load calibration data if available
calibration_data = load_calibration()
if calibration_data:
    try:
        sensor.mode = adafruit_bno055.CONFIG_MODE
        time.sleep(0.02)  # Wait for mode switch
        sensor.offsets_accelerometer = tuple(calibration_data["accelerometer"])
        sensor.offsets_magnetometer = tuple(calibration_data["magnetometer"])
        sensor.offsets_gyroscope = tuple(calibration_data["gyroscope"])
        print("Loaded existing calibration data")
    except (KeyError, ValueError) as e:
        print(f"Error loading calibration data: {e}")
        print("Using default calibration")
    finally:
        sensor.mode = adafruit_bno055.NDOF_MODE
        time.sleep(0.01)  # Wait for mode switch
else:
    print("No existing calibration data found. Using default calibration.")

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
# print("Initial orientation:")
# print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(initial_pitch, initial_roll, initial_yaw))

def imu_data_collection(prev_time, prev_acceleration, velocity_threshold=0.01):
    sensor_data = SensorData()
    
    # Read acceleration (unit: m/s^2)
    acceleration = sensor.linear_acceleration
    sensor_data.acceleration = acceleration
    
    # Read magnetic field strength (unit: microtesla)
    magnetic = sensor.magnetic
    sensor_data.magnetic = magnetic
    
    # Read gyroscope data (unit: degrees/sec)
    gyro = sensor.gyro
    sensor_data.gyroscope = gyro
    
    # Read gravity vector (unit: m/s^2)
    gravity = sensor.gravity
    sensor_data.gravity = gravity
    
    # Read Euler angles (unit: degrees)
    euler = sensor.euler
    sensor_data.euler = euler
    
    # Read quaternion (no unit)
    quaternion = sensor.quaternion
    sensor_data.quaternion = quaternion
    
    # Calculate the relative orientation if euler angles are available
    if euler is not None:
        relative_pitch = euler[1] - initial_pitch
        relative_roll = euler[2] - initial_roll
        relative_yaw = euler[0] - initial_yaw
        sensor_data.relative_orientation = (relative_pitch, relative_roll, relative_yaw)
    else:
        sensor_data.relative_orientation = (0, 0, 0)
    
    # Calculate velocity based on current and previous acceleration using trapezoidal method
    current_time = time.time()
    dt = current_time - prev_time
    velocity = tuple((prev_accel + accel) * dt / 2 for prev_accel, accel in zip(prev_acceleration, acceleration))
    
    # Apply threshold to velocity
    velocity = tuple(vel if abs(vel) > velocity_threshold else 0.0 for vel in velocity)
    sensor_data.velocity = velocity
    
    return sensor_data, velocity, current_time, acceleration

def plot_imu_data(timestamps, acceleration_data, integrated_acceleration_data, summed_velocity_data):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 12))
    
    # Plot acceleration data
    ax1.plot([t for t in timestamps], [a[0] for a in acceleration_data], label='X')
    ax1.plot([t for t in timestamps], [a[1] for a in acceleration_data], label='Y')
    # ax1.plot([t for t in timestamps], [a[2] for a in acceleration_data], label='Z')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration (m/s^2)')
    ax1.set_title('Raw Acceleration Data')
    ax1.legend()
    
    # Plot integrated acceleration data
    ax2.plot([t for t in timestamps], [a[0] for a in integrated_acceleration_data], label='X')
    ax2.plot([t for t in timestamps], [a[1] for a in integrated_acceleration_data], label='Y')
    # ax2.plot([t for t in timestamps], [a[2] for a in integrated_acceleration_data], label='Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Integrated Acceleration (Trapezoidal Method)')
    ax2.legend()
    
    # Plot summed velocity data
    ax3.plot([t for t in timestamps], [v[0] for v in summed_velocity_data], label='X')
    ax3.plot([t for t in timestamps], [v[1] for v in summed_velocity_data], label='Y')
    # ax3.plot([t for t in timestamps], [v[2] for v in summed_velocity_data], label='Z')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Summed Velocity')
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig('imu_plots.png', dpi=300, bbox_inches='tight')
    print("Plot disimpan sebagai 'imu_plots.png'")
    plt.close()

# OKE
def moving_average_filter(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# OKE
def exponential_moving_average_filter(data, alpha):
    filtered_data = np.zeros_like(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
    return filtered_data

# Bagus Banget
def median_filter(data, window_size):
    return scipy.signal.medfilt(data, window_size)

# Bagus Banget
def butterworth_low_pass_filter(data, cutoff_freq, sampling_freq, order):
    nyquist_freq = 0.5 * sampling_freq
    normalized_cutoff_freq = cutoff_freq / nyquist_freq
    b, a = scipy.signal.butter(order, normalized_cutoff_freq, btype='low', analog=False)
    return scipy.signal.lfilter(b, a, data)

# Bagus Banget
def savitzky_golay_filter(data, window_size, order):
    return scipy.signal.savgol_filter(data, window_size, order)

# Bagus Banget
def kalman_filter(data, Q, R):
    n_iter = len(data)
    sz = (n_iter,)
    x = np.zeros(sz)
    P = np.zeros(sz)
    K = np.zeros(sz)
    
    x[0] = data[0]
    P[0] = 1.0
    
    for k in range(1, n_iter):
        P[k] = P[k-1] + Q
        K[k] = P[k] / (P[k] + R)
        x[k] = x[k-1] + K[k] * (data[k] - x[k-1])
        P[k] = (1 - K[k]) * P[k]
    
    return x
