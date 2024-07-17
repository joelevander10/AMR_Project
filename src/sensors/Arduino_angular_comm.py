import serial
import time

def send_command(command):
    ser.write((command + "\n").encode())
    time.sleep(0.1)  # Give a short delay for Arduino to process the command

def main():
    global ser
    ser = serial.Serial('/dev/ttyACM1', 9600)  # Replace 'COM3' with the appropriate port
    time.sleep(2)  # Wait for the serial connection to initialize

    while True:
        command = input("Enter command (A, Z, M): ")
        if command == 'A' or command == 'a':
            angle = input("Enter the angle (-360 to 360): ")
            send_command(f"A{angle}")
        elif command == 'Z' or command == 'z':
            send_command("Z")
        elif command == 'M' or command == 'm':
            speed = input("Enter the speed (cm/s): ")
            duration = input("Enter the duration (s): ")
            send_command(f"M{speed} {duration}")
        else:
            print("Invalid command.")

if __name__ == '__main__':
    main()
