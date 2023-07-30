#!/usr/bin/env python3
# Author: Pratik M Tambe <enthusiasticgeek@gmail.com>
# Date: July 29, 2023
# Wit Motion WT9011DCL-BT5
# Basic capabilities
import serial

port = '/dev/ttyUSB0'
baud_rate = 115200

ser = serial.Serial(port, baud_rate, timeout=1)

def read_data_packet():
    # Read until two consecutive bytes 0x55 and 0x61 are found
    start_bytes = ser.read(1)
    while start_bytes != b'\x55':
        start_bytes = ser.read(1)

    # Check for the next byte to be 0x61
    if ser.read(1) == b'\x61':
        # Read 18 more bytes after 0x55 and 0x61
        data_packet = start_bytes + b'\x61' + ser.read(18)

        return data_packet
    else:
        # If not found, restart the search
        return read_data_packet()

def parse_data_packet(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    axL, axH, ayL, ayH, azL, azH, wxL, wxH, wyL, wyH, wzL, wzH, RollL, RollH, PitchL, PitchH, YawL, YawH = data_packet[2:20]

    # Calculate the acceleration values
    ax = ((axH << 8) | axL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)
    ay = ((ayH << 8) | ayL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)
    az = ((azH << 8) | azL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)

    # Calculate the angular velocity values
    wx = ((wxH << 8) | wxL) / 32768 * 2000  # Angular velocity in °/s
    wy = ((wyH << 8) | wyL) / 32768 * 2000  # Angular velocity in °/s
    wz = ((wzH << 8) | wzL) / 32768 * 2000  # Angular velocity in °/s

    # Calculate the roll, pitch, and yaw angles
    Roll = ((RollH << 8) | RollL) / 32768 * 180  # Roll angle in °
    Pitch = ((PitchH << 8) | PitchL) / 32768 * 180  # Pitch angle in °
    Yaw = ((YawH << 8) | YawL) / 32768 * 180  # Yaw angle in °

    return ax, ay, az, wx, wy, wz, Roll, Pitch, Yaw

try:
    while True:
        # Read the data packet (19 bytes) from the serial port
        data_packet = read_data_packet()

        # Parse the data packet to get the sensor values
        ax, ay, az, wx, wy, wz, Roll, Pitch, Yaw = parse_data_packet(data_packet)

        # Print the sensor values
        print(f"Acceleration (g): ({ax}, {ay}, {az}), Angular Velocity (°/s): ({wx}, {wy}, {wz}), Roll (°): {Roll}, Pitch (°): {Pitch}, Yaw (°): {Yaw}")

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
except Exception as e:
    print("Error:", e)
    ser.close()

