from flask import Flask, render_template, jsonify, request
from flask_sqlalchemy import SQLAlchemy
import math
import board
import busio
import adafruit_bno055
from datetime import datetime
from sensors.sensor_data import SensorData
import os
import serial
import time
import threading
import json

# Initialize Flask app
app = Flask(__name__)

# Configure the SQLite database
basedir = os.path.abspath(os.path.dirname(__file__))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'robot.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

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

# Initialize serial connection (you might need to adjust the port)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

sensor_data = SensorData()
data_lock = threading.Lock()

@app.route('/')
def index():
    return render_template('index.html')

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
    duration = 1000
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
        speeds = [-base_speed, base_speed/2, base_speed, base_speed/2]
    elif button == 7:  # Diag rev l
        speeds = [base_speed, base_speed/2, -base_speed, base_speed/2]
    elif button == 1:  # Diag FWD L
        speeds = [base_speed/2, base_speed, base_speed/2, -base_speed]
    elif button == 9:  # Diag rev r
        speeds = [base_speed/2, -base_speed, base_speed/2, base_speed]
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

@app.route('/send_command', methods=['POST'])
def send_command():
    data = request.json
    command = data.get('command')
    
    if not command:
        return jsonify({'error': 'No command provided'}), 400

    try:
        ser.write((command + "\n").encode())
        time.sleep(0.1)  # Give a short delay for Arduino to process the command
        
        # Read the response from Arduino
        response = ser.readline().decode().strip()
        
        return jsonify({'response': response})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=5050)
