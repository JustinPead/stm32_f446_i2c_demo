import serial as serial_lib  # Alias to avoid conflict with built-in `serial` library
import sqlite3
import struct
import sys
from datetime import datetime

# Configure the COM port
com_port = 'COM4'  # Adjust the port as per your system
baud_rate = 115200  # Set the baud rate according to your device
timeout = 1  # Timeout for serial communication

# Database setup
db_name = 'serial_data.db'

def setup_database():
    """Setup SQLite database with a table for storing serial data."""
    conn = sqlite3.connect(db_name)
    cursor = conn.cursor()
    # Create table if it doesn't exist
    cursor.execute('''CREATE TABLE IF NOT EXISTS data (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp TEXT,
                        device_id INTEGER,
                        sensor_id INTEGER,
                        value INTEGER
                      )''')
    conn.commit()
    conn.close()

def save_to_database(data):
    """Save data with timestamp to the database."""
    conn = sqlite3.connect(db_name)
    cursor = conn.cursor()
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print("Saving Data to DB")
    print(data)
    cursor.execute("INSERT INTO data (timestamp, device_id, sensor_id, value) VALUES (?, ?, ?, ?)", (timestamp, data[0],data[1],data[2]))
    conn.commit()
    conn.close()

def read_from_com():
    """Read data from the COM port."""
    try:
        # Try opening the serial port
        with serial_lib.Serial(com_port, baud_rate, timeout=timeout) as ser:
            print("Serial port opened successfully. Press Ctrl+C to exit.")
            while True:
                try:
                    # Read 7 bytes of data from the serial port
                    data = ser.read(7)
                    if data:
                        # Convert byte data to string and strip any null characters
                        array = decode_sensor_data(data)
                        print(array)
                        # Save the data to the database with a timestamp
                        save_to_database(array)
                except KeyboardInterrupt:
                    print("\nGracefully shutting down...")
                    # Optional: Add any cleanup code here
                    sys.exit(0)
    except serial_lib.SerialException as e:
        # Gracefully handle the error when COM port is not found or cannot be opened
        print(f"Error: Could not open the COM port '{com_port}'. {e}")
        sys.exit(1)
    except Exception as e:
        # Handle any other unexpected errors
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)

def decode_sensor_data(five_bytes):
    # Break down the bytes
    device_id = five_bytes[4]      # Device ID
    sensor_id = five_bytes[3]      # Sensor ID
    value_bytes = five_bytes[1:3]  # Two bytes for actual value
    crc = five_bytes[0]           # CRC check
    
    # Convert the 2 value bytes to a float
    # Assuming it's a 16-bit value in little-endian
    value = struct.unpack('<H', value_bytes)[0]
    
    # Print for debugging
    print(f"Device: {device_id}, Sensor: {sensor_id}")
    print(f"Value bytes: {hex(value_bytes[0])} {hex(value_bytes[1])}")
    print(f"CRC: {hex(crc)}")
    print(f"Decoded value: {value}")
    
    return [device_id, sensor_id, value]

if __name__ == "__main__":
    setup_database()
    read_from_com()