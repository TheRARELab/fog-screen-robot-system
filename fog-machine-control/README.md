# Fog Machine Control System 💨

This repository contains code for controlling a fog machine through a computer and Arduino. It includes C++ code for serial communication with Arduino and Arduino sketches to operate the fog machine and detect voltage.

There are also two ROS services to turn on and off the fog machine. See [ros_service_fog_control](ros_service_fog_control).

Note that there is duplicate code in [FogMachineController.ino](FogMachineController.ino) and [ros_service_fog_control/ros_service_fog_control.ino](ros_service_fog_control/ros_service_fog_control.ino). So code changes in one file should also be done in the other file.

## Repository Structure

- `SerialCommunication.c++`: C++ code for serial communication with Arduino.
- `FogMachineController.ino`: Arduino sketch to control the fog machine.

## Requirements

- **Operating System**: Ubuntu or macOS
- **Libraries**: Boost library (for C++)
- **Arduino IDE**: For compiling and uploading Arduino sketches
- **CMake**: For building C++ code

## Setup and Installation

1. **Boost Library Installation**:
   - Ubuntu: Install Boost via terminal:
     ```bash
     sudo apt-get install libboost-all-dev
     ```
   - macOS: Install Boost using Homebrew:
     ```bash
     brew install boost
     ```

2. **Arduino Setup**:
   - Install the Arduino IDE from the [Arduino website](https://www.arduino.cc/en/software).
   - Connect your Arduino to your computer.
   - Select your Arduino board.
   - Upload FogMachineController.ino to your Arduino.

## Compiling and Running

### ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

1. **Compile C++ Code**:
   - Navigate to the repository directory.
   - Run the following commands:
     ```bash
     mkdir build
     cd build
     cmake ..
     make
     ```
2. **Run C++ Code**:
    - Run the following command:
      ```bash
      ./SerialCommunication
      ```

## Usage
  - Type 'on' or 'ON' to turn on the fog machine and 'off' or 'OFF' to turn it off.
  - The Arduino will send a signal to the computer when the voltage is detected.
