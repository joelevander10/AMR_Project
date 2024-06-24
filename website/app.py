from flask import Flask, render_template, jsonify, request
from flask_sqlalchemy import SQLAlchemy
import math
from datetime import datetime
import os

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///control.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

class RobotState(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    x = db.Column(db.Float, nullable=False)
    y = db.Column(db.Float, nullable=False)
    rotation = db.Column(db.Float, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

class MovementHistory(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    action = db.Column(db.String(20), nullable=False)
    direction = db.Column(db.String(20), nullable=False)
    x = db.Column(db.Float, nullable=False)
    y = db.Column(db.Float, nullable=False)
    rotation = db.Column(db.Float, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

def create_database():
    if not os.path.exists('control.db'):
        with app.app_context():
            db.create_all()
        print("Database created.")

create_database()

robot_state = {
    'x': 240,
    'y': 135,
    'rotation': 0
}

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

@app.route('/update_control', methods=['POST'])
def update_control():
    data = request.json
    global robot_state

    speed = 5
    angle = math.radians(robot_state['rotation'])
    
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
            robot_state['rotation'] += 5
        elif data['direction'] == 'ccw':
            robot_state['rotation'] -= 5
        robot_state['rotation'] = robot_state['rotation'] % 360

    # Update the database
    new_state = RobotState(x=robot_state['x'], y=robot_state['y'], rotation=robot_state['rotation'])
    new_movement = MovementHistory(
        action=data['action'],
        direction=data['direction'],
        x=robot_state['x'],
        y=robot_state['y'],
        rotation=robot_state['rotation']
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

@app.route('/get_movement_history')
def get_movement_history():
    history = MovementHistory.query.order_by(MovementHistory.timestamp.desc()).limit(10).all()
    return jsonify([
        {
            'action': h.action,
            'direction': h.direction,
            'x': h.x,
            'y': h.y,
            'rotation': h.rotation,
            'timestamp': h.timestamp.isoformat()
        } for h in history
    ])

if __name__ == '__main__':
    app.run(debug=True)
