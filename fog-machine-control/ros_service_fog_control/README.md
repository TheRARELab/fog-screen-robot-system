Arduino code for two ROS services of turning the fog machine on and off.

## Requirements

- **Operating System**: Ubuntu 18.04 
- **ROS Version**: Melodic
- **ROS rosserial Package**: To install `rosserial` library on both Arduino IDE and ROS workstation, follow the instructions [here](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)  
- **Arduino IDE**

## How to run

1. **Running the service node**:
   - Compile and upload `ros_service_fog_control.ino` code to your Arduino micro-controller
   - In your terminal, run: `rosrun rosserial_python serial_node.py <port_name>`
     - "port_name" are usually like `ttyUSB0` or `ttyACM0`. Run `ls /dev/` to check which one your microcontroller is connected to.

2. In another tab,  call the `/fog_machine/turn_on` and `/fog_machine/turn_off` services to turn on and off the fog machine respectively.
```bash
rosservice call /fog_machine/turn_on
```
```bash
rosservice call /fog_machine/turn_off
```

# NOTE

When calling the services, you might get an error as seen in the picture below. It is a bug in `rosserial` package reported in [ros-drivers/rosserial#414](https://github.com/ros-drivers/rosserial/pull/414). A workaround solution is available here: [ros-drivers/rosserial#414 (comment)](https://github.com/ros-drivers/rosserial/pull/414#issuecomment-637702285)

![image](https://github.com/user-attachments/assets/bad651bc-9204-451c-90cb-b12810943f93)

