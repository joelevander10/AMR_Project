import time
import board
import busio
import adafruit_bno055
import json
import numpy as np

def is_sensor_connected_to_i2c(i2c):
    try:
        while not i2c.try_lock():
            pass
        i2c.unlock()
        return True
    except Exception:
        return False

def collect_calibration_data(sensor, num_samples=10):
    calibration_data = {
        "pitch": [],
        "roll": [],
        "yaw": []
    }

    for orientation in ["pitch", "roll", "yaw"]:
        print(f"\nCalibrating {orientation}:")
        print(f"Please rotate the sensor slowly around the {orientation} axis.")
        print(f"Collecting {num_samples} samples...")

        for i in range(num_samples):
            accel = sensor.acceleration
            gyro = sensor.gyro
            mag = sensor.magnetic
            calibration_data[orientation].append({
                "accelerometer": accel,
                "gyroscope": gyro,
                "magnetometer": mag
            })
            print(f"Sample {i+1}/{num_samples} collected")
            time.sleep(1)  # Wait for 1 second between samples

        print(f"{orientation.capitalize()} calibration complete!")

    return calibration_data

def save_calibration(calibration_data, filename="bno055_calibration_detailed.json"):
    with open(filename, "w") as f:
        json.dump(calibration_data, f, indent=2)
    print(f"Calibration data saved to {filename}")

# Initialize the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Check if the sensor is connected to the I2C bus
if not is_sensor_connected_to_i2c(i2c):
    print("BNO055 sensor not detected on the I2C bus. Please check the wiring and connection.")
    exit(1)

print("BNO055 sensor detected on the I2C bus")

# Initialize the BNO055 sensor
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("Starting detailed calibration process...")

# Collect calibration data
calibration_data = collect_calibration_data(sensor)

# Save calibration data
save_calibration(calibration_data)

print("\nCalibration complete!")
print("Calibration data saved. You can now use this data in your main program.")

# Example of how to use the calibration data
print("\nExample of calculating offsets:")
for orientation in ["pitch", "roll", "yaw"]:
    accel_offset = np.mean([sample["accelerometer"] for sample in calibration_data[orientation]], axis=0)
    gyro_offset = np.mean([sample["gyroscope"] for sample in calibration_data[orientation]], axis=0)
    mag_offset = np.mean([sample["magnetometer"] for sample in calibration_data[orientation]], axis=0)
    
    print(f"\n{orientation.capitalize()} offsets:")
    print(f"Accelerometer: {accel_offset}")
    print(f"Gyroscope: {gyro_offset}")
    print(f"Magnetometer: {mag_offset}")

print("\nTo use these offsets in your main program, subtract them from your sensor readings.")
