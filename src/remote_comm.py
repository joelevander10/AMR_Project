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

            if left_y > 0:
                # Forward movement
                fl = 500
                br = -500
                rl = -500
                bl = 500
            elif left_y < 0:
                # Backward movement
                fl = -500
                br = 500
                rl = 500
                bl = -500

            if left_x < 0:
                # Strafe left
                fl = 500
                br = 500
                rl = -500
                bl = -500
            elif left_x > 0:
                # Strafe right
                fl = -500
                br = -500
                rl = 500
                bl = 500

            if right_x < 0:
                # Rotate left
                fl = 500
                br = 500
                rl = 500
                bl = 500
            elif right_x > 0:
                # Rotate right
                fl = -500
                br = -500
                rl = -500
                bl = -500

            print(f"FL: {fl}, BR: {br}, RL: {rl}, BL: {bl}")

            # Send command to Arduino
            send_command(fl, br, rl, bl)
        else:
            # Stop the motors if RB trigger is not pressed
            send_command(0, 0, 0, 0)
        time.sleep(1)
if __name__ == "__main__":
    main()

