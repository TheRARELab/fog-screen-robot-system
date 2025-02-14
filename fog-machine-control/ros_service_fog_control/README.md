# Fog Machine ROS Services

This directory provides Arduino code and ROS services for turning a fog machine on and off. The code is configured for an Arduino Nano Every using specific pin assignments; if using a different microcontroller, please update the pin definitions in the `.ino` file.

## Requirements

- **Arduino IDE**: With rosserial libraries installed ([instructions](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup))
- **rosserial**: Needed for ROS–Arduino communication

## Usage

1. **Upload**
   - Open `ros_service_fog_control.ino` in the Arduino IDE and upload it to your Arduino Nano Every.

2. **Run rosserial**
   - In a terminal, run:
     ```bash
     rosrun rosserial_python serial_node.py /dev/<port_name>
     ```
   - Replace `<port_name>` (e.g., `ttyUSB0` or `ttyACM0`) as appropriate.

3. **Call Services**
   - In another terminal, call the services to turn the fog machine on or off:
     ```bash
     rosservice call /fog_machine/turn_on
     rosservice call /fog_machine/turn_off
     ```
