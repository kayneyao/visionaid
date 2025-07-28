#!/usr/bin/env python3
"""
Simple serial logger for LIS2MDL & LSM6DSL sensor data.
Starts dumping to CSV when it sees "MODE,RAW" and continues until "DONE".

Edit SERIAL_PORT and BAUDRATE below, then run:
    python log_sensors.py
"""

import serial
import csv

# ——— Configuration ———
SERIAL_PORT = "/dev/ttyACM0"   # your serial port
BAUDRATE    = 230400
OUTPUT_CSV  = "sensors_log.csv"
# ————————————————————

def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=None)
    print(f"Listening on {SERIAL_PORT} @ {BAUDRATE} baud. Waiting for MODE,RAW…")

    logging = False
    with open(OUTPUT_CSV, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if not logging:
                if line.upper() == "MODE,RAW":
                    logging = True
                    print("MODE,RAW detected—starting log.")
                continue

            if line.upper() == "DONE":
                print("DONE detected—stopping log.")
                break

            # write CSV row
            writer.writerow(line.split(","))
            csvfile.flush()

    ser.close()
    print(f"Data saved to {OUTPUT_CSV}")

main()