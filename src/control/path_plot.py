import time
import serial
import math
import matplotlib
matplotlib.use('Agg')  # Set the backend to 'Agg' for non-interactive plotting
import matplotlib.pyplot as plt

def get_motor_speeds(command):
    speeds = {
        'maju': (-500, 500, 500, -500),
        'mundur': (500, -500, -500, 500),
        'kiri': (500, 500, -500, -500),
        'kanan': (-500, -500, 500, 500),
        'serong_kiri_depan': (0, 500, 0, -500),
        'serong_kiri_belakang': (500, 0, -500, 0),
        'serong_kanan_depan': (-500, 0, 500, 0),
        'serong_kanan_belakang': (0, -500, 0, 500),
        'putar_kiri': (500, 500, 500, 500),
        'putar_kanan': (-500, -500, -500, -500),
    }
    return speeds.get(command, (0, 0, 0, 0))

def send_command(command):
    print(command)
    ser.write(command.encode())

def plot_robot_path(path):
    x, y = zip(*path)
    
    plt.figure(figsize=(6, 6))
    plt.plot(x, y, '-', linewidth=2)
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start')
    plt.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
    
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Robot Path')
    
    x_range = max(x) - min(x)
    y_range = max(y) - min(y)
    margin = 0.2 * max(x_range, y_range)
    plt.xlim(min(x) - margin, max(x) + margin)
    plt.ylim(min(y) - margin, max(y) + margin)
    
    plt.legend()
    plt.grid(True)
    plt.savefig('robot_path.png', dpi=300, bbox_inches='tight')
    print("Plot disimpan sebagai 'robot_path.png'")
    plt.close()

def main():
    distance_per_second = 0.236  # m/s
    rotation_per_second = math.pi / 4  # 90 degrees per second

    x = 0
    y = 0
    orientation = 0
    path = [(0, 0)]

    try:
        while True:
            command = input("Masukkan perintah (maju, mundur, kiri, kanan, serong_kiri_depan, serong_kiri_belakang, serong_kanan_depan, serong_kanan_belakang, putar_kiri, putar_kanan): ")
            
            if command == 'quit':
                break
            
            motor_speeds = get_motor_speeds(command)
            
            if motor_speeds == (0, 0, 0, 0):
                print("Perintah tidak valid.")
                continue
            
            send_command(f"{motor_speeds[0]} {motor_speeds[1]} {motor_speeds[2]} {motor_speeds[3]}\n")
            
            if command == 'maju':
                x -= distance_per_second * math.cos(orientation)
                y += distance_per_second * math.sin(orientation)
            elif command == 'mundur':
                x += distance_per_second * math.cos(orientation)
                y -= distance_per_second * math.sin(orientation)
            elif command == 'kanan':
                x += distance_per_second * math.sin(orientation)
                y += distance_per_second * math.cos(orientation)
            elif command == 'kiri':
                x -= distance_per_second * math.sin(orientation)
                y -= distance_per_second * math.cos(orientation)
            elif command == 'serong_kiri_depan':
                x -= distance_per_second / math.sqrt(2) * (math.cos(orientation) - math.sin(orientation))
                y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) + math.cos(orientation))
            elif command == 'serong_kiri_belakang':
                x += distance_per_second / math.sqrt(2) * (math.cos(orientation) + math.sin(orientation))
                y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) - math.cos(orientation))
            elif command == 'serong_kanan_depan':
                x -= distance_per_second / math.sqrt(2) * (math.cos(orientation) + math.sin(orientation))
                y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) - math.cos(orientation))
            elif command == 'serong_kanan_belakang':
                x += distance_per_second / math.sqrt(2) * (math.cos(orientation) - math.sin(orientation))
                y -= distance_per_second / math.sqrt(2) * (math.sin(orientation) + math.cos(orientation))
            elif command == 'putar_kiri':
                orientation -= rotation_per_second
            elif command == 'putar_kanan':
                orientation += rotation_per_second
            
            path.append((x, y))
            print(f"Posisi: ({x:.3f}, {y:.3f}), Orientasi: {math.degrees(orientation):.2f} derajat")
            time.sleep(1)  # Delay between commands
    
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna.")
    
    finally:
        plot_robot_path(path)

if __name__ == "__main__":
    # Initialize serial communication
    ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace '/dev/ttyACM0' with the appropriate serial port
    main()
