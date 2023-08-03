#!/usr/bin/env python3
# Author: Pratik M Tambe <enthusiasticgeek@gmail.com>
# Date: July 29, 2023
# Wit Motion WT9011DCL-BT5
# Basic capabilities with GUI


import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib
import serial
import serial.tools.list_ports


import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *


import platform  # Import the platform module


# Initialize Pygame
pygame.init()

# Set the window size
window_size = (800, 600)
display = pygame.display.set_mode(window_size, DOUBLEBUF | OPENGL)

# Call glutInit to initialize GLUT
glutInit()

# Set up the OpenGL perspective
gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 50.0)

# Set the initial camera position
glTranslatef(0.0, 0.0, -5)

def draw_text_3d_1(text, position):
    font = GLUT_BITMAP_9_BY_15
    glRasterPos3f(*position)
    for char in text:
        glutBitmapCharacter(font, ctypes.c_int(ord(char)))

def draw_text_3d(text, position, color=(1.0, 1.0, 1.0)):
    font = GLUT_BITMAP_9_BY_15
    glRasterPos3f(*position)
    glColor3f(*color)  # Set the text color (default is white)
    for char in text:
        glutBitmapCharacter(font, ctypes.c_int(ord(char)))

def draw_cubie(roll, yaw, pitch):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    # Clear the screen and the depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Draw x, y, and z arrows
    glLineWidth(1.0)  # Set the line width for the axes
    glBegin(GL_LINES)
    
    # Red for x-axis
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0.5, 0, 0)

    # Green for y-axis
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0.5, 0)

    # Blue for z-axis
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 0.5)

    glEnd()

    draw_text_3d(f"[0,0]", (0, 0, 0))
    # Draw pitch, roll, and yaw texts above x, y, and z axes
    draw_text_3d(f"Pitch: {pitch:.2f}", (0.6, 0, 0))
    draw_text_3d(f"Roll: {roll:.2f}", (0, 0.6, 0))
    draw_text_3d(f"Yaw: {yaw:.2f}", (0, 0, 0.6))

    # Rotate the cube using the provided roll, yaw, and pitch values
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Set the reference point (0, 0, 0) and apply rotations around x, y, and z axes
    glTranslatef(0.0, 0.0, 0.0)
    glRotatef(roll, 1, 0, 0)  # Rotate around x-axis
    glRotatef(yaw, 0, 1, 0)   # Rotate around y-axis
    glRotatef(pitch, 0, 0, 1) # Rotate around z-axis

    # Define the vertices of the cube (scaled down by 0.5 in each dimension)
    vertices = [
        [0.5, 0.5, -0.5],
        [0.5, -0.5, -0.5],
        [-0.5, -0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [0.5, 0.5, 0.5],
        [0.5, -0.5, 0.5],
        [-0.5, -0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ]

    # Define the edges of the cube using vertex indices
    edges = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
    )

    # Draw the cube
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

    # Update the display
    pygame.display.flip()
    pygame.time.wait(10)


def draw_cubie1(roll, yaw, pitch):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    # Clear the screen and the depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Draw x, y, and z arrows
    glBegin(GL_LINES)
    glColor3f(1.0, 0.0, 0.0)  # Red for x-axis
    glVertex3f(0, 0, 0)
    glVertex3f(1, 0, 0)

    glColor3f(0.0, 1.0, 0.0)  # Green for y-axis
    glVertex3f(0, 0, 0)
    glVertex3f(0, 1, 0)

    glColor3f(0.0, 0.0, 1.0)  # Blue for z-axis
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 1)
    glEnd()


    # Draw pitch, roll, and yaw texts above x, y, and z axes
    draw_text_3d(f"Pitch: {pitch:.2f}", (1.2, 0, 0))
    draw_text_3d(f"Roll: {roll:.2f}", (0, 1.2, 0))
    draw_text_3d(f"Yaw: {yaw:.2f}", (0, 0, 1.2))

    # Rotate the cube using the provided roll, yaw, and pitch values
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Set the reference point (0, 0, 0) and apply rotations around x, y, and z axes
    glTranslatef(0.0, 0.0, 0.0)
    glRotatef(roll, 1, 0, 0)  # Rotate around x-axis
    glRotatef(yaw, 0, 1, 0)   # Rotate around y-axis
    glRotatef(pitch, 0, 0, 1) # Rotate around z-axis

    # Define the vertices of the cube
    vertices = [
        [1, 1, -1],
        [1, -1, -1],
        [-1, -1, -1],
        [-1, 1, -1],
        [1, 1, 1],
        [1, -1, 1],
        [-1, -1, 1],
        [-1, 1, 1],
    ]

    # Define the edges of the cube using vertex indices
    edges = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
    )

    # Draw the cube
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

    # Update the display
    pygame.display.flip()
    pygame.time.wait(10)

def read_data_packet(start_byte=b'\x51'):
    # Read until two consecutive bytes 0x55 and start_byte are found
    start_bytes = ser.read(1)
    while start_bytes != b'\x55':
        start_bytes = ser.read(1)

    # Check for the next byte to be start_byte
    if ser.read(1) == start_byte:
        # Read 18 more bytes after 0x55 and start_byte
        data_packet = start_bytes + start_byte + ser.read(9)
        return data_packet
    else:
        # If not found, restart the search
        return read_data_packet(start_byte)

# Create a function to update the serial port list
def update_serial_port_list():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    combo_port.get_model().clear()
    for port in ports:
        combo_port.append_text(port)

# Create a function to update the serial port list on Linux
def update_serial_port_list_linux():
    try:
        ports = [port.device for port in serial.tools.list_ports.comports() if "ttyUSB" in port.device]
        combo_port.get_model().clear()
        for port in ports:
            combo_port.append_text(port)
    except Exception as e:
        print("Error:", e)

# Create a function to update the serial port list on Windows
def update_serial_port_list_windows():
    try:
        ports = [port.device for port in serial.tools.list_ports.comports() if "COM" in port.device]
        combo_port.get_model().clear()
        for port in ports:
            combo_port.append_text(port)
    except Exception as e:
        print("Error:", e)


ser = None  # Declare ser as a global variable
reading_active = False  # Variable to track if sensor data reading is active
timeout_id = None  # Initialize timeout_id as None
timeout_id = 0  # Initialize timeout_id to 0

def on_stop_button_clicked(button):
    global reading_active, timeout_id
    reading_active = False
    if timeout_id != 0:
        GLib.source_remove(timeout_id)
        timeout_id = 0  # Reset timeout_id to 0 after removing the timeout

def on_start_button_clicked(button):
    global ser, reading_active, timeout_id
    if reading_active == True:
        return
    port = combo_port.get_active_text()
    baud_rate = int(combo_baud_rate.get_active_text())

    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
    except Exception as e:
        print("Error:", e)
        return

    # Stop any existing timeout before starting a new one
    if not reading_active and timeout_id != 0:
        GLib.source_remove(timeout_id)
        timeout_id = 0  # Reset timeout_id to 0 after removing the timeout

    # Start updating sensor data periodically
    reading_active = True
    timeout_id = GLib.timeout_add(100, update_sensor_data)


def update_sensor_data():
    global ser, reading_active
    try:
        if ser is not None and reading_active:

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

            # Convert the bytes to hexadecimal representation
            hex_data = data_packet_angular_output.hex()
 
            # Print the hexadecimal data
            # print(hex_data)

            # Get the byte length of the data_packet
            data_length = len(data_packet_angular_output)

            # Print the hexadecimal data and the byte length
            print(f"Hex Data: {hex_data}, Byte Length: {data_length}")

            if data_packet_angular_output is not None and data_packet_angular_velocity is not None and data_packet_acceleration is not None:

                # Parse the data packet to get the sensor values
                # ax, ay, az, wx, wy, wz, Roll, Pitch, Yaw = parse_data_packet(data_packet_angular_output)

                # Update the label with the sensor values (formatted to 3 decimal places with different colors)
                label.set_markup(
                    "Acceleration (g) ["
                    "<span foreground='red'>x</span>, "
                    "<span foreground='green'>y</span>, "
                    "<span foreground='blue'>z</span>]: "
                    "(<span foreground='red'>{:.3f}</span>, <span foreground='green'>{:.3f}</span>, <span foreground='blue'>{:.3f}</span>),\n\n"
                    "Angular Velocity (°/s) ["
                    "<span foreground='red'>x</span>, "
                    "<span foreground='green'>y</span>, "
                    "<span foreground='blue'>z</span>]: "
                    "(<span foreground='red'>{:.3f}</span>, <span foreground='green'>{:.3f}</span>, <span foreground='blue'>{:.3f}</span>),\n\n"
                    "Roll (°): {:.3f},\n\n Pitch (°): {:.3f},\n\n Yaw (°): {:.3f}"
                    .format(ax, ay, az, wx, wy, wz, Roll, Pitch, Yaw)
                )


    except Exception as e:
        print("Error:", e)

    return reading_active  # Continue periodic updates if reading_active is True

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

    draw_cubie(int(Roll), int(Yaw), int(Pitch))

    return Roll, Pitch, Yaw, v_ao, sum_ao

# Create a GTK window
class SensorDataWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="Sensor Data Reader")
        self.set_border_width(10)

        # Set the minimum window size to 200x400
        self.set_default_size(600, 400)

        # Create a grid layout
        grid = Gtk.Grid()
        self.add(grid)

        # Create the serial port selection menu
        global combo_port
        combo_port = Gtk.ComboBoxText()
        # Check the platform to decide which function to use for updating the serial port list
        if platform.system() == "Linux":
            update_serial_port_list_linux()
        elif platform.system() == "Windows":
            update_serial_port_list_windows()
        combo_port.set_entry_text_column(0)
        combo_port.set_active(0)
        combo_port.connect("focus-in-event", lambda x, y: update_serial_port_list())
        grid.attach(combo_port, 0, 0, 1, 1)

        # Create the baud rate selection menu
        global combo_baud_rate
        combo_baud_rate = Gtk.ComboBoxText()
        # Add various baud rates to the drop-down menu
        baud_rates = ["9600", "115200", "38400", "57600", "19200", "57600"]
        for rate in baud_rates:
            combo_baud_rate.append_text(rate)
        combo_baud_rate.set_active(0)  # Set a default baud rate (e.g., 115200)
        grid.attach(combo_baud_rate, 1, 0, 1, 1)

        # Create a label to display the sensor data
        global label
        label = Gtk.Label()
        label.set_line_wrap(True)
        grid.attach(label, 0, 2, 2, 1)

        # Create a Start button
        start_button = Gtk.Button(label="Start Reading")
        start_button.connect("clicked", on_start_button_clicked)
        grid.attach(start_button, 0, 1, 2, 1)

        # Create a Stop button
        stop_button = Gtk.Button(label="Stop Reading")
        stop_button.connect("clicked", on_stop_button_clicked)
        grid.attach(stop_button, 2, 0, 1, 1)

win = SensorDataWindow()
win.connect("destroy", Gtk.main_quit)
win.show_all()

Gtk.main()
