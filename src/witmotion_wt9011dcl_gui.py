#!/usr/bin/env python3
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
import platform  # Import the platform module

def read_data_packet():
    try:
        # Read until two consecutive bytes 0x55 and 0x61 are found
        start_bytes = ser.read(1)
        while start_bytes != b'\x55':
            start_bytes = ser.read(1)
        # Check for the next byte to be 0x61
        if ser.read(1) == b'\x61':
            # Read 17 more bytes after 0x55 and 0x61
            data_packet = start_bytes + b'\x61' + ser.read(18)
            return data_packet
        else:
            # If not found, restart the search
            return read_data_packet()
    except Exception as e:
        print("Error:", e)
        return None

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
            # Read the data packet (19 bytes) from the serial port
            data_packet = read_data_packet()
 
            # Convert the bytes to hexadecimal representation
            hex_data = data_packet.hex()
 
            # Print the hexadecimal data
            # print(hex_data)

            # Get the byte length of the data_packet
            data_length = len(data_packet)

            # Print the hexadecimal data and the byte length
            print(f"Hex Data: {hex_data}, Byte Length: {data_length}")



            if data_packet is not None:


                # Parse the data packet to get the sensor values
                ax, ay, az, wx, wy, wz, Roll, Pitch, Yaw = parse_data_packet(data_packet)

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
        combo_baud_rate.set_active(1)  # Set a default baud rate (e.g., 115200)
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
