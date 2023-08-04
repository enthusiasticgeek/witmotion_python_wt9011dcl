#!/usr/bin/env python3
# Author: Pratik M Tambe <enthusiasticgeek@gmail.com>
# Date: Aug 2, 2023
# Wit Motion WT9011DCL-BT5
# Advanced capabilities
# Usage: sudo ./witmotion_wt901cttl.py --enable_acceleration --enable_angular_output --enable_<parameter1> --enable_<parameter2> ....
import serial
import time
import argparse
import sys

#Change the Port as necessary
port = '/dev/ttyUSB0'
baud_rate = 9600

ser = serial.Serial(port, baud_rate, timeout=1)

def read_data_packet(start_byte=b'\x51', timeout=1):
    # Set the start time for timeout calculation
    start_time = time.time()

    # Read until two consecutive bytes 0x55 and start_byte are found
    start_bytes = ser.read(1)
    while start_bytes != b'\x55':
        # Check for timeout
        if time.time() - start_time > timeout:
            print("Timeout while waiting for 0x55")
            return None
        start_bytes = ser.read(1)

    # Check for the next byte to be start_byte
    while ser.read(1) != start_byte:
        # Check for timeout
        if time.time() - start_time > timeout:
            print(f"Timeout while waiting for {start_byte}")
            return None

    # Read 9 more bytes after 0x55 and start_byte
    data_packet = start_bytes + start_byte + ser.read(9)
    return data_packet

def parse_data_packet_time(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    YY, MM, DD, HH, MN, SS, MSL, MSH, Sum = data_packet[2:11]

    # Calculate the Millisec
    msec_time = ((MSH << 8) | MSL)  # Millisec
    # Checksum
    sum_time = Sum

    return YY, MM, DD, HH, MN, SS, msec_time, sum_time

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

def parse_data_packet_magnetic_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    HxL, HxH, HyL, HyH, HzL, HzH, TL, TH, Sum = data_packet[2:11]

    # Calculate the Hx, Hy, Hz
    Hx = ((HxH << 8) | HxL)
    Hy = ((HyH << 8) | HyL)
    Hz = ((HzH << 8) | HzL)

    # Calculate the temperature values
    temp_mo = ((TH << 8) | TL) / 100
    # Checksum
    sum_mo = Sum

    return Hx, Hy, Hz, temp_mo, sum_mo

def parse_data_packet_port_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    D0L, D0H, D1L, D1H, D2L, D2H, D3L, D3H, Sum = data_packet[2:11]

    # Calculate the D0, D1, D2, D3
    D0 = ((D0H << 8) | D0L)
    D1 = ((D1H << 8) | D1L)
    D2 = ((D2H << 8) | D2L)
    D3 = ((D3H << 8) | D3L)

    # Checksum
    sum_po = Sum

    return D0, D1, D2, D3, sum_po

def parse_data_packet_pressure_altitude_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    P0, P1, P2, P3, H0, H1, H2, H3, Sum = data_packet[2:11]

    Pa = ((P3<<24)|(P2<<16)|(P1<<8)|P0)
    Cm = ((H3<<24)|(H2<<16)|(H1<<8)|H0)

    # Checksum
    sum_pa = Sum

    return Pa, Cm, sum_pa


def parse_data_packet_lat_long_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    Lon0, Lon1, Lon2, Lon3, Lat0, Lat1, Lat2, Lat3, Sum = data_packet[2:11]

    Lon = ((Lon3<<24)|(Lon2<<16)|(Lon1<<8)|Lon0)
    Lat = ((Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0)

    # Checksum
    sum_ll = Sum

    return Lat, Lon, sum_ll

def parse_data_packet_gps_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    gps_HL, gps_HH, gps_YL, gps_YH, gps_v0, gps_v1, gps_v2, gps_v3, Sum = data_packet[2:11]

    # Calculate the gps
    gps_alt = ((gps_HH << 8) | gps_HL)/10
    gps_head = ((gps_YH << 8) | gps_YL)/100
    gps_gnd_speed = ((gps_v3<<24)|(gps_v2<<16)|(gps_v1<<8)|gps_v0)/1000

    # Checksum
    sum_gps = Sum

    return gps_alt, gps_head, gps_gnd_speed, sum_gps

def parse_data_packet_quartenion_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    Q0L, Q0H, Q1L, Q1H, Q2L, Q2H, Q3L, Q3H, Sum = data_packet[2:11]

    q0 = ((Q0H<<8)|Q0L)/32768
    q1 = ((Q1H<<8)|Q1L)/32768
    q2 = ((Q2H<<8)|Q2L)/32768
    q3 = ((Q3H<<8)|Q3L)/32768

    # Checksum
    sum_qo = Sum

    return q0, q1, q2, q3, sum_qo

def parse_data_packet_gps_position_accuracy_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    SNL, SNH, PDOPL, PDOPH, HDOPL, HDOPH, VDOPL, VDOPH, Sum = data_packet[2:11]

    sat_num = ((SNH<<8)|SNL)
    pos_acc = ((PDOPH<<8)|PDOPL)/100
    hor_pre = ((HDOPH<<8)|HDOPL)/100
    ver_pre = ((VDOPH<<8)|VDOPL)/100

    # Checksum
    sum_gpa = Sum

    return sat_num, pos_acc, hor_pre, ver_pre, sum_gpa

def parse_data_packet_register_value_output(data_packet):
    # Unpack the data_packet based on the format described in the datasheet
    R1L, R1H, R2L, R2H, R3L, R3H, R4L, R4H, Sum = data_packet[2:11]

    R1 = ((R1H << 8) | R1L)
    R2 = ((R2H << 8) | R2L)
    R3 = ((R3H << 8) | R3L)
    R4 = ((R4H << 8) | R4L)

    # Checksum
    sum_rv = Sum

    return R1, R2, R3, R4, sum_rv


parser = argparse.ArgumentParser(description="Enable/Disable Functions Using Flags.")
parser.add_argument("-t", "--enable_time", action="store_true", help="Enable Time")
parser.add_argument("-a", "--enable_acceleration", action="store_true", help="Enable Acceleration")
parser.add_argument("-v", "--enable_angular_velocity", action="store_true", help="Enable Angular Velocity")
parser.add_argument("-o", "--enable_angular_output", action="store_true", help="Enable Angular Output")
parser.add_argument("-m", "--enable_magnetic_output", action="store_true", help="Enable Magnetic Output")
parser.add_argument("-p", "--enable_port_output", action="store_true", help="Enable Port Output")
parser.add_argument("-u", "--enable_pressure_altitude_output", action="store_true", help="Enable Pressure/Altitude Output")
parser.add_argument("-l", "--enable_lat_long_output", action="store_true", help="Enable Latitude/Longitude Output")
parser.add_argument("-g", "--enable_gps_output", action="store_true", help="Enable GPS Output")
parser.add_argument("-q", "--enable_quartenion_output", action="store_true", help="Enable Quartenion Output")
parser.add_argument("-c", "--enable_gps_position_accuracy_output", action="store_true", help="Enable GPS Position Accuracy Output")
parser.add_argument("-r", "--enable_register_value_output", action="store_true", help="Enable Register Value Output")
args = parser.parse_args()
# Check if any argument is provided. If none are given, print the help message.
if not any(vars(args).values()):
   parser.print_help()
   sys.exit()



try:
    while True:
        
        if args.enable_time == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_time = read_data_packet(b'\x50')
                if data_packet_time is not None:

                        # Parse the data packet to get the sensor values
                        YY, MM, DD, HH, MN, SS, msec_time, sum_time = parse_data_packet_time(data_packet_time)

                        # Print the sensor values
                        print(f"Time: ({YY}, {MM}, {DD}), {HH}, {MN}, {SS}, {msec_time}, {sum_time}")
        
        if args.enable_acceleration == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_acceleration = read_data_packet(b'\x51')
                if data_packet_acceleration is not None: 
                        # Parse the data packet to get the sensor values
                        ax, ay, az, temp_acc, sum_acc = parse_data_packet_acceleration(data_packet_acceleration)

                        # Print the sensor values
                        print(f"Acceleration (g): ({ax}, {ay}, {az}), {temp_acc} (°C), {sum_acc}")

        if args.enable_angular_velocity == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_angular_velocity = read_data_packet(b'\x52')
                if data_packet_angular_velocity is not None:
                        # Parse the data packet to get the sensor values
                        wx, wy, wz, vol_av, sum_av = parse_data_packet_angular_velocity(data_packet_angular_velocity)

                        # Print the sensor values
                        print(f"Angular Velocity (°/s): ({wx}, {wy}, {wz}), {vol_av} (V), {sum_av}")

        if args.enable_angular_output == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_angular_output = read_data_packet(b'\x53')
                if data_packet_angular_output is not None:
                        # Parse the data packet to get the sensor values
                        Roll, Pitch, Yaw, v_ao, sum_ao = parse_data_packet_angular_output(data_packet_angular_output)

                        # Print the sensor values
                        print(f"Angular Output: Roll (°): {Roll}, Pitch (°): {Pitch}, Yaw (°): {Yaw}, {v_ao} (V), {sum_ao}")

        if args.enable_magnetic_output == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_magnetic_output = read_data_packet(b'\x54')
                if data_packet_magnetic_output is not None:
                        # Parse the data packet to get the sensor values
                        Hx, Hy, Hz, temp_mo, sum_mo = parse_data_packet_magnetic_output(data_packet_magnetic_output)

                        # Print the sensor values
                        print(f"Magnetic Output: Hx: {Hx}, Hy: {Hy}, Hz: {Hz}, {temp_mo} (°C), {sum_mo}")

        if args.enable_port_output == True:
                # Read the data packet (9 bytes) from the serial port
                data_packet_port_output = read_data_packet(b'\x55')
                if data_packet_port_output is not None:
                        # Parse the data packet to get the sensor values
                        D0, D1, D2, D3, sum_po = parse_data_packet_port_output(data_packet_port_output)

                        # Print the sensor values
                        print(f"Port Output: D0: {D0}, D1: {D1}, D2: {D2}, D3: {D3}, {sum_po}")

        if args.enable_pressure_altitude_output == True:
                # Read the data packet (9 bytes) from the serial pressure_altitude
                data_packet_pressure_altitude_output = read_data_packet(b'\x56')
                if data_packet_pressure_altitude_output is not None:
                        # Parse the data packet to get the sensor values
                        Pa, Cm, sum_pa = parse_data_packet_pressure_altitude_output(data_packet_pressure_altitude_output)

                        # Print the sensor values
                        print(f"Pressure/Altitude Output: Pressure (Pa): {Pa}, Altitude (cm): {Cm}, {sum_pa}")

        if args.enable_lat_long_output == True:
                # Read the data packet (9 bytes) from the serial lat_long
                data_packet_lat_long_output = read_data_packet(b'\x57')
                if data_packet_lat_long_output is not None:
                        # Parse the data packet to get the sensor values
                        Lat, Lon, sum_ll = parse_data_packet_lat_long_output(data_packet_lat_long_output)

                        # Print the sensor values
                        print(f"Latitude/Longitude Output: Lat: {Lat}, Long: {Lon}, {sum_ll}")

        if args.enable_gps_output == True:
                # Read the data packet (9 bytes) from the serial gps
                data_packet_gps_output = read_data_packet(b'\x58')
                if data_packet_gps_output is not None:
                        # Parse the data packet to get the sensor values
                        gps_alt, gps_head, gps_gnd_speed, sum_gps = parse_data_packet_gps_output(data_packet_gps_output)

                        # Print the sensor values
                        print(f"GPS Data Output: GPS Altitude (m): {gps_alt}, GPS heading (°): {gps_head}, GPS ground speed (km/h): {gps_gnd_speed}, {sum_gps}")

        if args.enable_quartenion_output == True:
                # Read the data packet (9 bytes) from the serial quartenion
                data_packet_quartenion_output = read_data_packet(b'\x59')
                if data_packet_quartenion_output is not None:
                        # Parse the data packet to get the sensor values
                        q0, q1, q2, q3, sum_qo  = parse_data_packet_quartenion_output(data_packet_quartenion_output)

                        # Print the sensor values
                        print(f"Quartenion Output: Q0: {q0}, Q1: {q1}, Q2: {q2}, Q3: {q3}, {sum_qo}")

        if args.enable_gps_position_accuracy_output == True:
                # Read the data packet (9 bytes) from the serial gps_position_accuracy
                data_packet_gps_position_accuracy_output = read_data_packet(b'\x5A')
                if data_packet_gps_position_accuracy_output is not None:
                        # Parse the data packet to get the sensor values
                        sat_num, pos_acc, hor_pre, ver_pre, sum_gpa  = parse_data_packet_gps_position_accuracy_output(data_packet_gps_position_accuracy_output)

                        # Print the sensor values
                        print(f"GPS Positional Accuracy Output: Satellites: {sat_num}, Positional Accuracy (): {pos_acc}, Horizontal Precision (): {hor_pre}, Vertical Precision (): {ver_pre}, {sum_gpa}")

        if args.enable_register_value_output == True:
                # Read the data packet (9 bytes) from the serial register_value
                data_packet_register_value_output = read_data_packet(b'\x5F')
                if data_packet_register_value_output is not None:
                        # Parse the data packet to get the sensor values
                        r1, r2, r3, r4, sum_rv  = parse_data_packet_register_value_output(data_packet_register_value_output)

                        # Print the sensor values
                        print(f"Register Values Output: R1: {r1}, R2: {r2}, R3: {r3}, R4: {r4}, {sum_rv}")



except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
except Exception as e:
    print("Error:", e)
    ser.close()

