from flask import Flask, render_template, jsonify, request
from flask_sqlalchemy import SQLAlchemy
import math
from datetime import datetime
import os
import serial
import time
import atexit

# Initialize serial connection (you might need to adjust the port)
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize


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

def get_latest_robot_state():
    with app.app_context():
        latest_state = RobotState.query.order_by(RobotState.timestamp.desc()).first()
        if latest_state:
            return {
                'x': latest_state.x,
                'y': latest_state.y,
                'orientation': latest_state.orientation
            }
    return {'x': 0, 'y': 0, 'orientation': 0}

def init_db_and_state():
    with app.app_context():
        db.create_all()
    return get_latest_robot_state()

# Initialize robot state
robot_state = init_db_and_state()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/map.html')
def map_page():
    return render_template('map.html')

@app.route('/telemetry.html')
def telemetry_page():
    return render_template('telemetry.html')

@app.route('/manual.html')
def manual_page():
    return render_template('manual.html')

@app.route('/parameter.html')
def parameter_page():
    return render_template('parameter.html')

@app.route('/get_position')
def get_position():
    with app.app_context():
        # Get the two most recent entries
        recent_states = RobotState.query.order_by(RobotState.timestamp.desc()).limit(2).all()
        
        if len(recent_states) < 2:
            return jsonify({'error': 'Not enough data points'}), 400
        
        current_state = recent_states[0]
        prev_state = recent_states[1]
        
        x = current_state.x - prev_state.x
        y = current_state.y - prev_state.y
        orientation = current_state.orientation - prev_state.orientation
        
        # Normalize orientation to be between -180 and 180 degrees
        orientation = (orientation + 180) % 360 - 180
    
    # Here you would typically send x, y, and orientation to Arduino
    # For now, we'll just return the values as JSON
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
    
    # Base template for mecanum wheel speeds
    base_speed = 500
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
    
    # Convert speeds to strings and add a default run duration (e.g., 1000 ms)
    speed_strings = [str(speed) for speed in speeds]
    run_duration = "1000"  # 1 second, adjust as needed
    
    # Prepare the command string
    #command = " ".join(speed_strings) + "\n"
    fl = speeds[0]
    bl = speeds[1]
    fr = speeds[2]
    br = speeds[3]
    command = f"{fl} {bl} {fr} {br}\n"
    print(command)
    # Send the command to Arduino
    ser.write(command.encode())
    
    # Wait for acknowledgment and speed values from Arduino
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response.startswith("SPEEDS "):
                received_speeds = response.split()[1:]
                move = calculate_movement(speed_strings, received_speeds)
                return jsonify({
                    'sent_speeds': speeds,
                    'received_speeds': [int(speed) for speed in received_speeds],
                    'movement': move
                })
            elif response == "ERROR":
                return jsonify({'error': 'Arduino detected invalid speed values'}), 400
            else:
                return jsonify({'error': 'Unknown response from Arduino'}), 500
    
    # If we reach here, there was no response from Arduino
    return jsonify({'error': 'No response from Arduino'}), 500

@app.route('/update_control', methods=['POST'])
def update_control():
    data = request.json
    global robot_state

    speed = 5
    angle = math.radians(robot_state['orientation'])
    
    if data['action'] == 'move':
        if data['direction'] == 'forward':
            robot_state['x'] += speed * math.sin(angle)
            robot_state['y'] -= speed * math.cos(angle)
        elif data['direction'] == 'reverse':
            robot_state['x'] -= speed * math.sin(angle)
            robot_state['y'] += speed * math.cos(angle)
        elif data['direction'] == 'left':
            robot_state['x'] -= speed * math.cos(angle)
            robot_state['y'] -= speed * math.sin(angle)
        elif data['direction'] == 'right':
            robot_state['x'] += speed * math.cos(angle)
            robot_state['y'] += speed * math.sin(angle)
        elif data['direction'] == 'diagFwdL':
            robot_state['x'] += speed * 0.7071 * (math.sin(angle) - math.cos(angle))
            robot_state['y'] -= speed * 0.7071 * (math.cos(angle) + math.sin(angle))
        elif data['direction'] == 'diagFwdR':
            robot_state['x'] += speed * 0.7071 * (math.sin(angle) + math.cos(angle))
            robot_state['y'] -= speed * 0.7071 * (math.cos(angle) - math.sin(angle))
        elif data['direction'] == 'diagRevL':
            robot_state['x'] -= speed * 0.7071 * (math.sin(angle) + math.cos(angle))
            robot_state['y'] += speed * 0.7071 * (math.cos(angle) - math.sin(angle))
        elif data['direction'] == 'diagRevR':
            robot_state['x'] -= speed * 0.7071 * (math.sin(angle) - math.cos(angle))
            robot_state['y'] += speed * 0.7071 * (math.cos(angle) + math.sin(angle))
    elif data['action'] == 'rotate':
        if data['direction'] == 'cw':
            robot_state['orientation'] += 5
        elif data['direction'] == 'ccw':
            robot_state['orientation'] -= 5
        robot_state['orientation'] = robot_state['orientation'] % 360

    with app.app_context():
        # Update the database
        new_state = RobotState(x=robot_state['x'], y=robot_state['y'], orientation=robot_state['orientation'])
        new_movement = MovementHistory(
            action=data['action'],
            direction=data['direction'],
            x=robot_state['x'],
            y=robot_state['y'],
            orientation=robot_state['orientation']
        )
        db.session.add(new_state)
        db.session.add(new_movement)
        db.session.commit()

    print("Updated robot state:", robot_state)
    return jsonify(robot_state)

@app.route('/get_control')
def get_control():
    print("Current robot state:", robot_state)
    return jsonify(robot_state)

if __name__ == '__main__':
    app.run(host='0.0.0.0')
