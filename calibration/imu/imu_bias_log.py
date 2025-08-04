import serial
import csv
import time

# ——— CONFIGURATION ———
PORT     = '/dev/ttyACM0'  # e.g. 'COM3' on Windows
BAUD     = 230400
OUTPUT   = 'imu_log.csv'
DURATION = None            # seconds, or None to log until Ctrl-C
# ————————————————

# Open serial port
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)                # give MCU time to reset
ser.reset_input_buffer()     # clear any startup messages

# Kick off the ESP32 logging
ser.write(b'S')
time.sleep(0.1)

# Open CSV for writing
with open(OUTPUT, 'w', newline='') as f:
    writer = csv.writer(f)
    # write our own header
    writer.writerow(['t','ax','ay','az','gx','gy','gz','mx','my','mz'])

    start_time = time.time()
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            # Decode & strip
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # Print every line received
            print(line)

            # Split into fields and write if correct length
            parts = [field.strip() for field in line.split(',')]
            if len(parts) == 10:
                writer.writerow(parts)
                f.flush()

            # Check for duration limit
            if DURATION is not None and (time.time() - start_time) > DURATION:
                print(f"Finished logging after {DURATION} seconds.")
                break

    except KeyboardInterrupt:
        print("Logging interrupted by user.")
    finally:
        ser.close()
        print(f"Serial port closed, data saved to {OUTPUT}")
