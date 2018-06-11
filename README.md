
Simple Utility to publish voltages to ROS at 100 ms from a Force Sensitive Resistor connected to a Phidget 2/2/2 interface kit.
Used on a Baxter Research Robot to recieve haptic feedback from the gripper

##Assumptions:

1. Only one Phidget device is connected to the computer
2. Input is an analogue voltage input
3. Phidget driver for your OS is installed
4. Ros kinetic and ROSpy are installed

## Usage:

Download the script or clone the repository.

from a temrinal: `chmod +x ~/path/to/baxter/src/phidget_force.py`
Run the script with `./path/to/phidget_force.py`

You can also listen with `rostopic echo GripperForceSensor`
