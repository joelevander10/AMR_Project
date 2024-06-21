import pygame
import serial
import time
import math

# Initialize Pygame
pygame.init()

# Check for connected joysticks
joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks: {joystick_count}")

# Initialize the Xbox 360 controller
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")
else:
    print("No joystick found.")
    quit()

# Set up serial connection to the Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)  

def calculate_mecanum_speeds(left_speed_x, left_speed_y, rotation_speed):
    # Hitung kecepatan untuk setiap motor stepper berdasarkan arah gerak diagonal dan orientasi
    # (implementasi perhitungan kecepatan tergantung pada konfigurasi roda mecanum wheel)
    # Contoh perhitungan kecepatan untuk konfigurasi roda mecanum wheel tertentu
    fl_speed = left_speed_y + left_speed_x + rotation_speed
    fr_speed = left_speed_y - left_speed_x - rotation_speed
    rl_speed = left_speed_y - left_speed_x + rotation_speed
    rr_speed = left_speed_y + left_speed_x - rotation_speed
    
    speeds = [fl_speed, fr_speed, rl_speed, rr_speed]
    
    # Normalisasi kecepatan menjadi 0 atau 500 sps
    for i in range(len(speeds)):
        speeds[i] = 500 if speeds[i] > 0 else -500 if speeds[i] < 0 else 0
    
    return speeds

# Game loop
while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()
        elif event.type == pygame.JOYAXISMOTION:
            # Analog wheel kiri untuk menentukan arah gerak diagonal
            if event.axis == 0:  # Left analog stick horizontal
                left_speed_x = int(event.value * 500)
            elif event.axis == 1:  # Left analog stick vertical
                left_speed_y = int(event.value * -500)
            
            # Analog wheel kanan untuk menentukan orientasi (memutar AMR)
            elif event.axis == 2:  # Right analog stick horizontal
                rotation_speed = int(event.value * 500)
            
        elif event.type == pygame.JOYBUTTONDOWN:
            # Tombol trigger RB sebagai trigger
            if event.button == 5:  # RB button
                trigger_pressed = True
        elif event.type == pygame.JOYBUTTONUP:
            if event.button == 5:  # RB button
                trigger_pressed = False
    
    # Kirim perintah ke Arduino hanya jika tombol trigger RB ditekan
    if trigger_pressed:
        # Hitung kecepatan untuk setiap motor stepper berdasarkan arah gerak diagonal dan orientasi
        speeds = calculate_mecanum_speeds(left_speed_x, left_speed_y, rotation_speed)
        
        # Kirim kecepatan ke Arduino
        speeds_str = " ".join(map(str, speeds)) + "\n"
        ser.write(speeds_str.encode())
    else:
        # Jika tombol trigger RB tidak ditekan, kirim perintah untuk menghentikan motor
        ser.write("0 0 0 0\n".encode())
    
    # Update the screen (if needed)
    pygame.display.flip()
