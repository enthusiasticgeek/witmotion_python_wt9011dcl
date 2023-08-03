#!/usr/bin/env python3
# Author: Pratik M Tambe <enthusiasticgeek@gmail.com>
# Date: July 29, 2023
# Wit Motion WT9011DCL-BT5
# Basic capabilities
import serial

port = '/dev/ttyUSB0'
baud_rate = 9600

ser = serial.Serial(port, baud_rate, timeout=1)

def read_data_packet(start_byte=b'\x51'):
    # Read until two consecutive bytes 0x55 and start_byte are found
    start_bytes = ser.read(1)
    while start_bytes != b'\x55':
        start_bytes = ser.read(1)

    # Check for the next byte to be start_byte
    if ser.read(1) == start_byte:
        # Read 9 more bytes after 0x55 and start_byte
        data_packet = start_bytes + start_byte + ser.read(9)
        return data_packet
    else:
        # If not found, restart the search
        return read_data_packet(start_byte)


#0x55 0x51 
#AxL AxH AyL AyH AzL AzH TL TH SUM

def parse_data_packet_acceleration(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    axL, axH, ayL, ayH, azL, azH, TL, TH, Sum = data_packet[2:11]

    # Calculate the acceleration values
    ax = ((axH << 8) | axL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)
    ay = ((ayH << 8) | ayL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)
    az = ((azH << 8) | azL) / 32768 * 16  # Acceleration in g (9.8 m/s^2)

    # Calculate the temperature values
    temp_acc = ((TH << 8) | TL) / 100  # Temperature
    # Checksum
    sum_acc = Sum

    return ax, ay, az, temp_acc, sum_acc

def parse_data_packet_angular_velocity(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    wxL, wxH, wyL, wyH, wzL, wzH, volL, volH, Sum = data_packet[2:11]

    # Calculate the angular velocity values
    wx = ((wxH << 8) | wxL) / 32768 * 2000  # Angular velocity in °/s
    wy = ((wyH << 8) | wyL) / 32768 * 2000  # Angular velocity in °/s
    wz = ((wzH << 8) | wzL) / 32768 * 2000  # Angular velocity in °/s

    # Calculate the voltage values
    vol_av = ((volH << 8) | volL) / 100  # Voltage
    # Checksum
    sum_av = Sum

    return wx, wy, wz, vol_av, sum_av

def parse_data_packet_angular_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    RollL, RollH, PitchL, PitchH, YawL, YawH, vL, vH, Sum = data_packet[2:11]

    # Calculate the roll, pitch, and yaw angles
    Roll = ((RollH << 8) | RollL) / 32768 * 180  # Roll angle in °
    Pitch = ((PitchH << 8) | PitchL) / 32768 * 180  # Pitch angle in °
    Yaw = ((YawH << 8) | YawL) / 32768 * 180  # Yaw angle in °

    # Calculate the version values
    v_ao = ((vH << 8) | vL) / 100  # Version
    # Checksum
    sum_ao = Sum

    return Roll, Pitch, Yaw, v_ao, sum_ao



try:
    while True:
        # Read the data packet (9 bytes) from the serial port
        data_packet_acceleration = read_data_packet(b'\x51')

        # Parse the data packet to get the sensor values
        ax, ay, az, temp_acc, sum_acc = parse_data_packet_acceleration(data_packet_acceleration)

        # Print the sensor values
        print(f"Acceleration (g): ({ax}, {ay}, {az}), {temp_acc} (°C), {sum_acc}")

        # Read the data packet (9 bytes) from the serial port
        data_packet_angular_velocity = read_data_packet(b'\x52')

        # Parse the data packet to get the sensor values
        wx, wy, wz, vol_av, sum_av = parse_data_packet_angular_velocity(data_packet_angular_velocity)

        # Print the sensor values
        print(f"Angular Velocity (°/s): ({wx}, {wy}, {wz}), {vol_av} (V), {sum_av}")

        # Read the data packet (9 bytes) from the serial port
        data_packet_angular_output = read_data_packet(b'\x53')

        # Parse the data packet to get the sensor values
        Roll, Pitch, Yaw, v_ao, sum_ao = parse_data_packet_angular_output(data_packet_angular_output)

        # Print the sensor values
        print(f"Angular Output: Roll (°): {Roll}, Pitch (°): {Pitch}, Yaw (°): {Yaw}, {v_ao} (V), {sum_ao}")



except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
except Exception as e:
    print("Error:", e)
    ser.close()

