# witmotion_python_wt9011dcl

WitMotion WT9011DCL GUI /CLI Python code

## Dependencies (Tested On Ubuntu)

Install Python 3.8+ and pip3 installed (Most Linux distributions come with this preinstalled)

    sudo apt-get update && sudo apt install python3 python3-pip
    sudo apt install libgtk-3-dev -y
    sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0 -y
    sudo apt install python3-opengl -y
    sudo apt-get install python3-dev python3-pip libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libgl1-mesa-dev libgles2-mesa-dev -y
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

        sudo ./witmotion_wt9011dcl.py

#### GUI Method:

        sudo ./witmotion_wt9011dcl_gui.py

![alt text]( https://github.com/enthusiasticgeek/witmotion_python_wt9011dcl/blob/main/wt9011dcl.png "example output")
