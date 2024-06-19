import serial
import time

def is_valid_input(speeds, run_duration):
    if len(speeds) != 4:
        return False
    for speed in speeds:
        if not speed.lstrip("-").isdigit() or int(speed) < -1000 or int(speed) > 1000:
            return False
    if not run_duration.isdigit() or int(run_duration) <= 0:
        return False
    return True

def calculate_error(sent_speeds, received_speeds):
    errors = []
    for sent, received in zip(sent_speeds, received_speeds):
        error = abs(int(sent) - int(received))
        errors.append(error)
    return errors

def main():
    # Set up serial connection to the Arduino
    ser = serial.Serial('/dev/ttyACM0', 9600)  
    #time.sleep(2)  # Wait for the serial connection to initialize

    print("Masukkan kecepatan dalam langkah per detik (0-1000 sps) untuk masing-masing motor stepper (dipisahkan oleh spasi) dan durasi putaran dalam milidetik:")
    while True:
        user_input = input()
        input_parts = user_input.split()
        if len(input_parts) != 5:
            print("Input tidak valid. Harap masukkan 4 nilai kecepatan yang valid (-1000-1000) dan durasi putaran dalam milidetik.")
            continue
        sent_speeds = input_parts[:4]
        run_duration = input_parts[4]
        if not is_valid_input(sent_speeds, run_duration):
            print("Input tidak valid. Harap masukkan 4 nilai kecepatan yang valid (-1000-1000) dan durasi putaran dalam milidetik.")
            continue

        # Send the speeds and run duration to the Arduino
        speeds_str = " ".join(sent_speeds) + " " + run_duration + "\n"
        ser.write(speeds_str.encode())

        # Wait for acknowledgment and speed values from Arduino
        while True:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response.startswith("SPEEDS "):
                    received_speeds = response.split()[1:]
                    errors = calculate_error(sent_speeds, received_speeds)
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

        time.sleep(0.1)  # Give a short delay before next input

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Terjadi kesalahan komunikasi serial: {e}")
    except KeyboardInterrupt:
        print("Program dihentikan oleh pengguna.")
