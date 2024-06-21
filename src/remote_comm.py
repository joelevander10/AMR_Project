import time
import pygame
import serial

# Initialize Pygame
pygame.init()

# Set up the joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialize serial communication
ser = serial.Serial('COM4', 9600)  # Replace 'COM4' with the appropriate serial port

def send_command(fl, br, rl, bl):
    command = f"{fl} {br} {rl} {bl}\n"
    print(command)
    ser.write(command.encode())

def main():
    is_rb_pressed = False
    left_y = 0
    left_x = 0
    right_x = 0
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 1:
                    # Left analog stick Y-axis for forward/backward movement
                    left_y = int(event.value * 500)
                elif event.axis == 0:
                    # Left analog stick X-axis for strafing
                    left_x = int(event.value * 500)
                elif event.axis == 2:
                    # Right analog stick X-axis for rotation
                    right_x = int(event.value * 500)
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 5:
                    # RB trigger
                    is_rb_pressed = True
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 5:
                    # RB trigger
                    is_rb_pressed = False

        if is_rb_pressed:
            # Determine movement directions based on analog stick values
            fl = 0
            br = 0
            rl = 0
            bl = 0

            # Diagonal movements
            if abs(left_y) > 250 and abs(left_x) > 250:
                if left_y < -250 and left_x > 250:
                    # Diagonal Front Right
                    fl, br, rl, bl = -500, 0, 500, 0
                elif left_y > 250 and left_x < -250:
                    # Diagonal Back Left
                    fl, br, rl, bl = 500, 0, -500, 0
                elif left_y > 250 and left_x > 250:
                    # Diagonal Back Right
                    fl, br, rl, bl = 0, -500, 0, -500
                elif left_y < -250 and left_x < -250:
                    # Diagonal Front Left
                    fl, br, rl, bl = 0, 500, 0, -500
            else:
                # Existing movements
                if left_y < -250:
                    # Forward movement
                    fl, br, rl, bl = 500, -500, -500, 500
                elif left_y > 250:
                    # Backward movement
                    fl, br, rl, bl = -500, 500, 500, -500
                elif left_x < -250:
                    # Strafe left
                    fl, br, rl, bl = 500, 500, -500, -500
                elif left_x > 250:
                    # Strafe right
                    fl, br, rl, bl = -500, -500, 500, 500

            # Rotation (overrides other movements)
            if abs(right_x) > 250:
                if right_x < -250:
                    # Rotate left
                    fl, br, rl, bl = 500, 500, 500, 500
                elif right_x > 250:
                    # Rotate right
                    fl, br, rl, bl = -500, -500, -500, -500

            print(f"FL: {fl}, BR: {br}, RL: {rl}, BL: {bl}")
            # Send command to Arduino
            send_command(fl, br, rl, bl)
        else:
            # Stop the motors if RB trigger is not pressed
            send_command(0, 0, 0, 0)

        time.sleep(0.1)  # Reduced sleep time for more responsive control

if __name__ == "__main__":
    main()
