from flask_sqlalchemy import SQLAlchemy
import math
import board
import busio
# import adafruit_bno055
from datetime import datetime
from sensors.sensor_data import SensorData
import os
import serial
import time
import threading
import json
from flask import Flask, render_template, jsonify, request, redirect, url_for, session, flash
from werkzeug.security import generate_password_hash, check_password_hash
import os

# Initialize Flask app
app = Flask(__name__)


# Configure the SQLite database
basedir = os.path.abspath(os.path.dirname(__file__))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'robot.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

app.config['SECRET_KEY'] = 'secret'  # Change this to a random secret key


# Initialize the database
db = SQLAlchemy(app)

# Define the database models
class RobotState(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    x = db.Column(db.Float)
    y = db.Column(db.Float)
    orientation = db.Column(db.Float)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

class MovementHistory(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    action = db.Column(db.String(20))
    direction = db.Column(db.String(20))
    x = db.Column(db.Float)
    y = db.Column(db.Float)
    orientation = db.Column(db.Float)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

class User(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password = db.Column(db.String(120), nullable=False)

# Initialize serial connection (you might need to adjust the port)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

sensor_data = SensorData()
data_lock = threading.Lock()

with app.app_context():
    db.create_all()
@app.route('/')
def home():
    if 'logged_in' in session:
        return redirect(url_for('index'))
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['uname']
        password = request.form['psw']
        
        user = User.query.filter_by(username=username).first()
        
        if user and check_password_hash(user.password, password):
            session['logged_in'] = True
            session['username'] = username
            flash('Logged in successfully.', 'success')
            return redirect(url_for('index'))
        else:
            flash('Invalid username or password', 'error')
    
    return render_template('login.html')

@app.route('/regisx', methods=['GET', 'POST'])
def register():
    if request.method == 'POST':
        username = request.form['uname']
        password = request.form['psw']
        
        existing_user = User.query.filter_by(username=username).first()
        if existing_user:
            flash('Username already exists', 'error')
            return redirect(url_for('register'))
        
        hashed_password = generate_password_hash(password)
        new_user = User(username=username, password=hashed_password)
        db.session.add(new_user)
        db.session.commit()
        
        flash('Account created successfully. Please log in.', 'success')
        return redirect(url_for('login'))
    
    return render_template('register.html')

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    session.pop('username', None)
    flash('Logged out successfully.', 'success')
    return redirect(url_for('login'))

@app.route('/index')
def index():
    if 'logged_in' not in session:
        flash('Please log in to access this page.', 'error')
        return redirect(url_for('login'))
    return render_template('index.html', username=session['username'])

@app.route('/map.html')
def map_page():
    return render_template('map.html')

@app.route('/telemetry.html')
def telemetry_page():
    with data_lock:
        return render_template('telemetry.html', sensor_data=sensor_data)

@app.route('/manual.html')
def manual_page():
    return render_template('manual.html')

@app.route('/parameter.html')
def parameter_page():
    return render_template('parameter.html')

@app.route('/get_position')
def get_position():
    with app.app_context():
        recent_states = RobotState.query.order_by(RobotState.timestamp.desc()).limit(2).all()
        
        if len(recent_states) < 2:
            return jsonify({'error': 'Not enough data points'}), 400
        
        current_state = recent_states[0]
        prev_state = recent_states[1]
    
        x = current_state.x - prev_state.x
        y = current_state.y - prev_state.y
        orientation = current_state.orientation - prev_state.orientation
        
        orientation = (orientation + 180) % 360 - 180
    
    return jsonify({'x': x, 'y': y, 'orientation': orientation})

def calculate_movement(sent_speeds, received_speeds):
    movement = []
    for sent, received in zip(sent_speeds, received_speeds):
        move = abs(int(sent) - int(received))
        movement.append(move)
    return movement

@app.route('/control_amr', methods=['POST'])
def control_amr():
    data = request.json
    button = data.get('button')
    
    base_speed = 500
    duration = 2000
    speeds = [0, 0, 0, 0]  # [front_left, front_right, rear_left, rear_right]
    
    if button == 2:  # Forward 
        speeds = [-base_speed, base_speed, base_speed, -base_speed]
    elif button == 8:  # Backward
        speeds = [base_speed, -base_speed, -base_speed, base_speed]
    elif button == 4:  # Left
        speeds = [base_speed, base_speed, -base_speed, -base_speed]
    elif button == 6:  # Right
        speeds = [-base_speed, -base_speed, base_speed, base_speed]
    elif button == 5:  # Stop
        speeds = [0, 0, 0, 0]
    elif button == 3:  # Diag FWD R
        speeds = [-base_speed, base_speed/base_speed, base_speed, base_speed/base_speed]
    elif button == 7:  # Diag rev l
        speeds = [base_speed, base_speed/base_speed, -base_speed, base_speed/base_speed]
    elif button == 1:  # Diag FWD L
        speeds = [base_speed/base_speed, base_speed, base_speed/base_speed, -base_speed]
    elif button == 9:  # Diag rev r
        speeds = [base_speed/base_speed, -base_speed, base_speed/base_speed, base_speed]
    elif button == 11:  # CW
        speeds = [-base_speed, -base_speed, -base_speed, -base_speed]
    elif button == 10:  # CCW
        speeds = [base_speed, base_speed, base_speed, base_speed]
    else:
        return jsonify({'error': 'Invalid button'}), 400
    
    fl, bl, fr, br = speeds
    command = f"{fl} {bl} {fr} {br} {duration}\n"
    print(command)
    ser.write(command.encode())
    
    response = read_from_serial()
    if response is None:
        return jsonify({'error': 'No response from Arduino'}), 500
    
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response.startswith("SPEEDS "):
                received_speeds = response.split()[1:]
                move = calculate_movement(speeds, received_speeds)
                return jsonify({
                    'sent_speeds': speeds,
                    'received_speeds': [int(speed) for speed in received_speeds],
                    'movement': move
                })
            elif response == "ERROR":
                return jsonify({'error': 'Arduino detected invalid speed values'}), 400
            else:
                return jsonify({'error': 'Unknown response from Arduino'}), 500
    
    return jsonify({'error': 'No response from Arduino'}), 500

@app.route('/update_control', methods=['POST'])
def update_control():
    data = request.json
    global robot_state

    distance_per_second = 5
    rotation_per_second = math.radians(5)
    angle = math.radians(robot_state['orientation'])
    
    if data['action'] == 'move':
        if data['direction'] == 'forward':
            robot_state['x'] -= distance_per_second * math.cos(angle)
            robot_state['y'] += distance_per_second * math.sin(angle)
        elif data['direction'] == 'reverse':
            robot_state['x'] += distance_per_second * math.cos(angle)
            robot_state['y'] -= distance_per_second * math.sin(angle)
        elif data['direction'] == 'left':
            robot_state['x'] -= distance_per_second * math.sin(angle)
            robot_state['y'] -= distance_per_second * math.cos(angle)
        elif data['direction'] == 'right':
            robot_state['x'] += distance_per_second * math.sin(angle)
            robot_state['y'] += distance_per_second * math.cos(angle)
        elif data['direction'] == 'diagFwdL':
            robot_state['x'] -= distance_per_second / math.sqrt(2) * (math.cos(angle) - math.sin(angle))
            robot_state['y'] -= distance_per_second / math.sqrt(2) * (math.sin(angle) + math.cos(angle))
        elif data['direction'] == 'diagFwdR':
            robot_state['x'] -= distance_per_second / math.sqrt(2) * (math.cos(angle) + math.sin(angle))
            robot_state['y'] -= distance_per_second / math.sqrt(2) * (math.sin(angle) - math.cos(angle))
        elif data['direction'] == 'diagRevL':
            robot_state['x'] += distance_per_second / math.sqrt(2) * (math.cos(angle) + math.sin(angle))
            robot_state['y'] -= distance_per_second / math.sqrt(2) * (math.sin(angle) - math.cos(angle))
        elif data['direction'] == 'diagRevR':
            robot_state['x'] += distance_per_second / math.sqrt(2) * (math.cos(angle) - math.sin(angle))
            robot_state['y'] -= distance_per_second / math.sqrt(2) * (math.sin(angle) + math.cos(angle))
    elif data['action'] == 'rotate':
        if data['direction'] == 'ccw':
            robot_state['orientation'] -= math.degrees(rotation_per_second)
        elif data['direction'] == 'cw':
            robot_state['orientation'] += math.degrees(rotation_per_second)
        robot_state['orientation'] = robot_state['orientation'] % 360

    print("Updated robot state:", robot_state)
    return jsonify(robot_state)

@app.route('/get_control')
def get_control():
    print("Current robot state:", robot_state)
    return jsonify(robot_state)

def read_from_serial(timeout=1):
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            return ser.readline().decode().strip()
    return None

def update_sensor_data():
    prev_time = time.time()
    prev_acceleration = (0.0, 0.0, 0.0)
    
    while True:
        with data_lock:
            global sensor_data
            sensor_data, velocity, prev_time, prev_acceleration = imu_data_collection(prev_time, prev_acceleration)
        time.sleep(0.1)  # Update at 10 Hz

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
# Initialize the I2C bus

def imu_data_collection(prev_time, prev_acceleration, velocity_threshold=0.01):
    global sensor_data
    
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

@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.json.get('command')
    if command:
        send_to_arduino(command)
        return jsonify({'success': True, 'message': f'Command sent: {command}'})
    else:
        return jsonify({'success': False, 'message': 'No command provided'}), 400
    
def send_to_arduino(command):
    ser.write((command + "\n").encode())
    time.sleep(0.1)  # Give a short delay for Arduino to process the command
    response = read_from_serial()
    print(f"Arduino response: {response}")
    # You can handle the response here if needed

def read_from_serial(timeout=1):
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            return ser.readline().decode().strip()
    return None

@app.route('/set_zero_orientation', methods=['POST'])
def set_zero_orientation():
    send_command("Z")
    response = read_from_serial()
    if response == "Robot set to zero position.":
        return jsonify({'success': True, 'message': 'Robot set to zero position.'})
    else:
        return jsonify({'success': False, 'message': 'Failed to set zero position.'})

@app.route('/execute_orientation', methods=['POST'])
def execute_orientation():
    angle = request.json.get('angle')
    if angle is None or not (-360 <= angle <= 360):
        return jsonify({'success': False, 'message': 'Invalid angle.'})
    
    send_command(f"A{angle}")
    response = read_from_serial()
    if response and "Robot rotated by" in response:
        return jsonify({'success': True, 'message': response})
    else:
        return jsonify({'success': False, 'message': 'Failed to rotate robot.'})

@app.route('/update_speed_and_duration', methods=['POST'])
def update_speed_and_duration():
    speed = request.json.get('speed')
    duration = request.json.get('duration')
    if speed is None or duration is None:
        return jsonify({'success': False, 'message': 'Invalid speed or duration.'})
    
    send_command(f"M{speed} {duration}")
    response = read_from_serial()
    if response and "Moving robot with speed" in response:
        return jsonify({'success': True, 'message': response})
    else:
        return jsonify({'success': False, 'message': 'Failed to update speed and duration.'})

if __name__ == '__main__':
    i2c = busio.I2C(board.SCL, board.SDA)

    # Check if the sensor is connected to the I2C bus
    if not is_sensor_connected_to_i2c(i2c):
        print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
        exit(1)
    print("BNO055 sensor detected on the I2C bus")

    # Initialize the BNO055 sensor
    # sensor = adafruit_bno055.BNO055_I2C(i2c)

    # Load calibration data if available
    # calibration_data = load_calibration()
    # if calibration_data:
    #     try:
    #         sensor.mode = adafruit_bno055.CONFIG_MODE
    #         time.sleep(0.02)  # Wait for mode switch
    #         sensor.offsets_accelerometer = tuple(calibration_data["accelerometer"])
    #         sensor.offsets_magnetometer = tuple(calibration_data["magnetometer"])
    #         sensor.offsets_gyroscope = tuple(calibration_data["gyroscope"])
    #         print("Loaded existing calibration data")
    #     except (KeyError, ValueError) as e:
    #         print(f"Error loading calibration data: {e}")
    #         print("Using default calibration")
    #     finally:
    #         sensor.mode = adafruit_bno055.NDOF_MODE
    #         time.sleep(0.01)  # Wait for mode switch
    # else:
    #     print("No existing calibration data found. Using default calibration.")

    # # Initialize the sensor
    # try:
    #     sensor.mode = adafruit_bno055.NDOF_MODE
    # except (RuntimeError, OSError) as e:
    #     print("Failed to initialize BNO055 sensor: {}".format(e))
    #     exit(1)
    # print("BNO055 sensor initialized")

    # Get the initial orientation
    # initial_euler = sensor.euler
    # initial_pitch, initial_roll, initial_yaw = initial_euler[1], initial_euler[2], initial_euler[0]

    sensor_thread = threading.Thread(target=update_sensor_data)
    sensor_thread.daemon = True
    sensor_thread.start()
    
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=5050)
