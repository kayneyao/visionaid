import serial
import json
import time

def read_serial_data(port, baud_rate, timeout):
    # Initialize serial port
    ser = serial.Serial(port, baud_rate, timeout=timeout)


    name = input()
    file = "./data/serial_data" + name + ".json"

    with open(file, 'w') as f:
        f.write("\n")
    
    try:
        while True:
            # Read a line from the serial port
            ser.write(b'g')
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                
                # Process the data (e.g., convert to JSON)
                data = {
                    "data": line
                }
                
                # Dump the data to a JSON file
                with open(file, 'a') as json_file:
                    json.dump(data, json_file)
                    json_file.write('\n')
                
                print(data)  # Optional: print the data to the console
                
    except KeyboardInterrupt:
        print("Stopping serial data reading.")
    
    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Set the serial port parameters
    port = 'COM3'  # Replace with your port name
    baud_rate = 9600  # Replace with your baud rate
    timeout = 1  # Read timeout in seconds
    
    read_serial_data(port, baud_rate, timeout)