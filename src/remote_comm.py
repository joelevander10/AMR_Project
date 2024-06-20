import serial
from inputs import get_gamepad

# Initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace '/dev/ttyACM0' with the appropriate serial port

def send_command(directions):
    command = " ".join(str(direction) for direction in directions) + "\n"
    ser.write(command.encode())

def main():
    while True:
        events = get_gamepad()
        for event in events:
            if event.code == 'ABS_Y':
                # Left analog stick Y-axis for forward/backward movement
                left_y = event.state
            elif event.code == 'ABS_X':
                # Left analog stick X-axis for strafing
                left_x = event.state
            elif event.code == 'ABS_RX':
                # Right analog stick X-axis for rotation
                right_x = event.state
            elif event.code == 'BTN_TR':
                # RB trigger
                is_rb_pressed = event.state

        if is_rb_pressed:
            # Determine movement directions based on analog stick values
            directions = [0, 0, 0, 0]  # [FL, FR, RR, RL]

            if left_y < 0:
                # Forward movement
                directions = [1, 1, 1, 1]
            elif left_y > 0:
                # Backward movement
                directions = [-1, -1, -1, -1]

            if left_x < 0:
                # Strafe left
                directions[0] = -1
                directions[1] = 1
                directions[2] = -1
                directions[3] = 1
            elif left_x > 0:
                # Strafe right
                directions[0] = 1
                directions[1] = -1
                directions[2] = 1
                directions[3] = -1

            if right_x < 0:
                # Rotate left
                directions[0] = -1
                directions[1] = -1
                directions[2] = 1
                directions[3] = 1
            elif right_x > 0:
                # Rotate right
                directions[0] = 1
                directions[1] = 1
                directions[2] = -1
                directions[3] = -1

            # Send command to Arduino
            send_command(directions)
        else:
            # Stop the motors if RB trigger is not pressed
            directions = [0, 0, 0, 0]
            send_command(directions)

if __name__ == "__main__":
    main()
