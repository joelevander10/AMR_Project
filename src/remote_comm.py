import time
import pygame
import serial

def is_valid_input(speeds):
    if len(speeds) != 4:
        return False
    for speed in speeds:
        if not speed.lstrip("-").isdigit() or int(speed) < -1000 or int(speed) > 1000:
            return False
    return True

def calculate_error(sent_speeds, received_speeds):
    errors = []
    for sent, received in zip(sent_speeds, received_speeds):
        error = abs(int(sent) - int(received))
        errors.append(error)
    return errors

def send_command(fl, br, rl, bl):
    speeds = [fl, br, rl, bl]
    if not is_valid_input(speeds):
        print("Input tidak valid. Harap masukkan 4 nilai kecepatan yang valid (-1000-1000).")
        return

    speeds_str = " ".join(map(str, speeds)) + "\n"
    ser.write(speeds_str.encode())

    # Wait for acknowledgment and speed values from Arduino
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response.startswith("SPEEDS "):
                received_speeds = response.split()[1:]
                errors = calculate_error(speeds, received_speeds)
                print("Nilai error komunikasi:")
                print(f"Stepper 1: {errors[0]} sps")
                print(f"Stepper 2: {errors[1]} sps")
                print(f"Stepper 3: {errors[2]} sps")
                print(f"Stepper 4: {errors[3]} sps")
                break
            elif response == "ERROR":
                print("Arduino mendeteksi nilai kecepatan yang tidak valid. Harap coba lagi.")
                break
            else:
                print("Respon tidak dikenal dari Arduino. Harap periksa koneksi serial.")
                break

def main():
    # Initialize Pygame
    pygame.init()

    # Set up the joystick
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Set up serial connection to the Arduino
    global ser
    ser = serial.Serial('COM4', 9600)  # Replace 'COM4' with the appropriate serial port

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

            if left_y < 0:
                # Forward movement
                fl = 500
                br = -500
                rl = -500
                bl = 500
            elif left_y > 0:
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

        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Terjadi kesalahan komunikasi serial: {e}")
    except KeyboardInterrupt:
        print("Program dihentikan oleh pengguna.")
