# witmotion_python_wt9011dcl

WitMotion WT9011DCL GUI (3D Cube) /CLI Python code

## Update:

WitMotion WT901CTTL GUI (3D Cube) /CLI Python code now available!

## Note: 

Currently the method to obtain parameters is using python's PySerial Library. We will explore BLE 5 option in the future.

## Dependencies (Tested On Ubuntu)

Install Python 3.8+ and pip3 installed (Most Linux distributions come with this preinstalled)

    sudo apt-get update && sudo apt install python3 python3-pip
    sudo apt install libgtk-3-dev -y
    sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0 -y
    sudo apt install python3-opengl -y
    sudo apt-get install python3-dev python3-pip libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libgl1-mesa-dev libgles2-mesa-dev -y
    sudo -H pip3 install pyserial
    sudo -H pip3 install pygame PyOpenGL PyOpenGL-accelerate

## Hardware Information/ Datasheets

https://witmotion-sensor.com/products/witmotion-wt9011dcl-bluetooth-5-0-accelerometer-gyroscope-angle-sensor-electronic-compass-magnetometer-inclinometer

https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0

https://drive.google.com/drive/folders/1I6sBC-8Q3_vtY-GrFDZbWJZJFk7UnNfO

## Communications Protocol:

https://drive.google.com/drive/folders/1cZ8Wjn0KKyztG4NRaTN-y4XcqHpFKz6m

## Usage:

Connect the WT9011DCL IMU via USB C to USB A cable to Ubuntu/Linux PC. 

#### CLI Method:

        cd src; 
        sudo ./witmotion_wt9011dcl.py

#### GUI Method:

        cd src; 
        sudo ./witmotion_wt9011dcl_gui.py


#### GUI 3D (OpenGL) Method:

        cd src; 
        sudo ./witmotion_wt9011dcl_gui_3d.py

![alt text]( https://github.com/enthusiasticgeek/witmotion_python_wt9011dcl/blob/main/wt9011dcl.png "example output")

## Note:

Initially the WT9011DCL did not spit out the data. However, I sent the hex string to factory reset the device. I also attached the accompanying BLE adapter dongle and ran a Bluetooth devices scan. Also, I added the user to dialout group on Ubuntu. I am not sure which of these events got the device to spit out data after disconnecting and reconnecting to the USB port. My guess is the one of latter two.

        echo -n -e '\xFF\xAA\x00\x01\x00' > /dev/ttyUSB0 | timeout 3 cat /dev/ttyUSB0 | hexdump -C

And

        sudo usermod -aG dialout ${USER}

The following may need to be run as well on some Linux systems.

        sudo chmod a+rw /dev/ttyUSB0

https://github.com/enthusiasticgeek/witmotion_python_wt9011dcl/assets/4500752/126575a5-4817-4e96-b9d3-13cef0ecf1b2


