import time
import serial
import math
import matplotlib
matplotlib.use('Agg')  # Set the backend to 'Agg' for non-interactive plotting
import matplotlib.pyplot as plt
import board
import busio
import adafruit_bno055
from sensor_data import SensorData
import signal
import json
import os
import numpy as np
import scipy
import scipy.signal
import threading
import curses
from queue import Queue

# Global variables
sensor = None
initial_pitch = initial_roll = initial_yaw = 0
start_time = time.time()
path = [(0, 0)]
avg_calibration = (0, 0, 0)
acceleration_data = []
integrated_acceleration_data = []
summed_velocity_data = []
timestamps = []
command = 'stop'
last_stop_time = time.time()
velocity_forced_zero = False
x = y = orientation = 0
prev_velocity = (0.0, 0.0, 0.0)
prev_time = time.time()
prev_acceleration = (0.0, 0.0, 0.0)

# Initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600)

# Initialize variables for robot control
distance_per_second = 0.236  # m/s
rotation_per_second = math.pi / 4  # 90 degrees per second

# Create a queue for thread communication
data_queue = Queue()

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

def get_motor_speeds(command):
    speeds = {
        'maju': (-500, 500, 500, -500),
        'mundur': (500, -500, -500, 500),
        'kiri': (500, 500, -500, -500),
        'kanan': (-500, -500, 500, 500),
        'serong_kiri_depan': (0, 500, 0, -500),
        'serong_kiri_belakang': (500, 0, -500, 0),
        'serong_kanan_depan': (-500, 0, 500, 0),
        'serong_kanan_belakang': (0, -500, 0, 500),
        'putar_kiri': (500, 500, 500, 500),
        'putar_kanan': (-500, -500, -500, -500),
        'stop': (0, 0, 0, 0)
    }
    return speeds.get(command, (0, 0, 0, 0))

def send_command(command):
    print(command)
    ser.write(command.encode())

def plot_robot_path(path):
    x, y = zip(*path)
    
    plt.figure(figsize=(6, 6))
    plt.plot(x, y, '-', linewidth=2)
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start')
    plt.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
    
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Robot Path')
    
    x_range = max(x) - min(x)
    y_range = max(y) - min(y)
    margin = 0.2 * max(x_range, y_range)
    plt.xlim(min(x) - margin, max(x) + margin)
    plt.ylim(min(y) - margin, max(y) + margin)
    
    plt.legend()
    plt.grid(True)
    plt.savefig('robot_path.png', dpi=300, bbox_inches='tight')
    print("Plot disimpan sebagai 'robot_path.png'")
    plt.close()

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
    ax1.plot(timestamps, [a[0] for a in acceleration_data], label='X')
    ax1.plot(timestamps, [a[1] for a in acceleration_data], label='Y')
    # ax1.plot(timestamps, [a[2] for a in acceleration_data], label='Z')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration (m/s^2)')
    ax1.set_title('Raw Acceleration Data')
    ax1.legend()
    
    # Plot integrated acceleration data
    ax2.plot(timestamps, [a[0] for a in integrated_acceleration_data], label='X')
    ax2.plot(timestamps, [a[1] for a in integrated_acceleration_data], label='Y')
    # ax2.plot(timestamps, [a[2] for a in integrated_acceleration_data], label='Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Integrated Acceleration (Trapezoidal Method)')
    ax2.legend()
    
    # Plot summed velocity data
    ax3.plot(timestamps, [v[0] for v in summed_velocity_data], label='X')
    ax3.plot(timestamps, [v[1] for v in summed_velocity_data], label='Y')
    # ax3.plot(timestamps, [v[2] for v in summed_velocity_data], label='Z')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Summed Velocity')
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig('imu_plots.png', dpi=300, bbox_inches='tight')
    print("Plot disimpan sebagai 'imu_plots.png'")
    plt.close()
    
def signal_handler(sig, frame):
    print("\nCtrl+C ditekan. Menyimpan plot dan keluar...")
    plot_imu_data(timestamps, acceleration_data, integrated_acceleration_data, summed_velocity_data)
    plot_robot_path(path)
    exit(0)

def median_filter(data, window_size):
    return scipy.signal.medfilt(data, window_size)

def butterworth_low_pass_filter(data, cutoff_freq, sampling_freq, order):
    nyquist_freq = 0.5 * sampling_freq
    normalized_cutoff_freq = cutoff_freq / nyquist_freq
    b, a = scipy.signal.butter(order, normalized_cutoff_freq, btype='low', analog=False)
    return scipy.signal.lfilter(b, a, data)

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

def imu_thread():
    global acceleration_data, integrated_acceleration_data, summed_velocity_data, timestamps, prev_velocity, prev_time, prev_acceleration, velocity_forced_zero

    while True:
        sensor_data, velocity, prev_time, prev_acceleration = imu_data_collection(prev_time, prev_acceleration)

        calibrated_acceleration = tuple(accel - cal for accel, cal in zip(sensor_data.acceleration, avg_calibration))

        filtered_acceleration = median_filter(calibrated_acceleration, window_size=5)
        filtered_acceleration = butterworth_low_pass_filter(filtered_acceleration, cutoff_freq=1.0, sampling_freq=115, order=4)
        filtered_acceleration = kalman_filter(filtered_acceleration, Q=0.001, R=0.1)
        
        data_queue.put(('acceleration', filtered_acceleration))
        data_queue.put(('velocity', velocity))

        cond_1 = time.time() - last_stop_time >= 0.5
        cond_2 = command == 'stop'
        cond_3 = all(abs(accel) <= 0.5 for accel in filtered_acceleration)

        if cond_1 and cond_2 and cond_3 and not velocity_forced_zero:
            velocity = (0.0, 0.0, 0.0)
            velocity_forced_zero = True
            data_queue.put(('message', "Velocity forced to zero."))
        
        acceleration_data.append(filtered_acceleration)

        if len(integrated_acceleration_data) == 0:
            velocity = (0.0, 0.0, 0.0)

        integrated_acceleration_data.append(velocity)
        summ_velocity = tuple(prev_vel + vel for prev_vel, vel in zip(prev_velocity, velocity))        
        data_queue.put(('sum_velocity', summ_velocity))
        summed_velocity_data.append(summ_velocity)
        timestamps.append(prev_time - start_time)

        prev_velocity = summed_velocity_data[-1]

        time.sleep(1/115)
        
def main_thread(screen):
    global command, last_stop_time, velocity_forced_zero, x, y, orientation, path

    while True:
        screen.clear()
        screen.addstr(0, 0, "Masukkan perintah: ")
        screen.refresh()
        command = screen.getstr().decode('utf-8')

        if command == 'quit':
            break

        motor_speeds = get_motor_speeds(command)
        send_command(f"{motor_speeds[0]} {motor_speeds[1]} {motor_speeds[2]} {motor_speeds[3]}\n")

        last_stop_time = time.time()
        velocity_forced_zero = False
        
        if command == 'maju':
            x -= distance_per_second * math.cos(orientation)
            y += distance_per_second * math.sin(orientation)
        elif command == 'mundur':
            x += distance_per_second * math.cos(orientation)
            y -= distance_per_second * math.sin(orientation)
        elif command == 'kanan':
            x += distance_per_second * math.sin(orientation)
            y += distance_per_second * math.cos(orientation)
        elif command == 'kiri':
            x -= distance_per_second * math.sin(orientation)
            y -= distance_per_second * math.cos(orientation)
        elif command == 'serong_kiri_depan':
            x -= distance_per_second / math.sqrt(2) * (math.cos(orientation) - math.sin(orientation))
            y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) + math.cos(orientation))
        elif command == 'serong_kiri_belakang':
            x += distance_per_second / math.sqrt(2) * (math.cos(orientation) + math.sin(orientation))
            y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) - math.cos(orientation))
        elif command == 'serong_kanan_depan':
            x -= distance_per_second / math.sqrt(2) * (math.cos(orientation) + math.sin(orientation))
            y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) - math.cos(orientation))
        elif command == 'serong_kanan_belakang':
            x += distance_per_second / math.sqrt(2) * (math.cos(orientation) - math.sin(orientation))
            y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) + math.cos(orientation))
        elif command == 'putar_kiri':
            orientation -= rotation_per_second
        elif command == 'putar_kanan':
            orientation += rotation_per_second

        path.append((x, y))
        screen.addstr(1, 0, f"Posisi: ({x:.3f}, {y:.3f}), Orientasi: {math.degrees(orientation):.2f} derajat")
        screen.refresh()
        time.sleep(1)  # Delay between commands

def display_thread(screen):
    while True:
        while not data_queue.empty():
            data_type, data = data_queue.get()
            if data_type == 'acceleration':
                screen.addstr(0, 0, f"Acceleration: {data}")
            elif data_type == 'velocity':
                screen.addstr(1, 0, f"Velocity: {data}")
            elif data_type == 'sum_velocity':
                screen.addstr(3, 0, f"Sum Velocity: {data}")
            elif data_type == 'message':
                screen.addstr(2, 0, data)
        screen.refresh()
        time.sleep(0.1)
        
def main(screen):
    global sensor, initial_pitch, initial_roll, initial_yaw, avg_calibration, start_time

    signal.signal(signal.SIGINT, signal_handler)

    i2c = busio.I2C(board.SCL, board.SDA)

    if not is_sensor_connected_to_i2c(i2c):
        print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
        exit(1)
    print("BNO055 sensor detected on the I2C bus")

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
    print("Initial orientation:")
    print("Pitch: {:.3f} Roll: {:.3f} Yaw: {:.3f}".format(initial_pitch, initial_roll, initial_yaw))

    # Initialize data storage lists
    acceleration_data = []
    integrated_acceleration_data = []
    summed_velocity_data = []
    timestamps = []

    print("Kalibrasi selama 10 detik...")
    calibration_start_time = time.time()
    calibration_data = []

    while time.time() - calibration_start_time < 10.0:  # Calibrate for 10 seconds
        acceleration = sensor.linear_acceleration
        calibration_data.append(acceleration)
        time.sleep(1/115)  # Maintain the sampling rate at 64 Hz

    # Calculate average calibration values
    avg_calibration = tuple(sum(axis) / len(axis) for axis in zip(*calibration_data))
    print(f"Nilai kalibrasi: {avg_calibration}")

    
    path = [(0, 0)]

    print("Pengumpulan data dan kontrol robot dimulai. Tekan Ctrl+C untuk berhenti.")

    start_time = time.time()

    imu_thread_handle = threading.Thread(target=imu_thread)
    imu_thread_handle.daemon = True
    imu_thread_handle.start()

    display_thread_handle = threading.Thread(target=display_thread, args=(screen,))
    display_thread_handle.daemon = True
    display_thread_handle.start()

    main_thread_handle = threading.Thread(target=main_thread, args=(screen,))
    main_thread_handle.start()

    main_thread_handle.join()

    # Apply moving average to the collected data
    window_size = min(2, len(acceleration_data) // 2)
    if window_size > 0:
        acceleration_data[:] = moving_average(acceleration_data, window_size)
        integrated_acceleration_data[:] = moving_average(integrated_acceleration_data, window_size)
        summed_velocity_data[:] = moving_average(summed_velocity_data, window_size)
        timestamps[:] = moving_average(timestamps, window_size)

    plot_imu_data(timestamps, acceleration_data, integrated_acceleration_data, summed_velocity_data)
    plot_robot_path(path)

    # Close the serial connection
    ser.close()

if __name__ == "__main__":
    curses.wrapper(main)
