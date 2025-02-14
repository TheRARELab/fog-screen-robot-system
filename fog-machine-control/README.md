# Custom Fog Machine Controller

![PCB Controller-Wider Medium](https://github.com/user-attachments/assets/32641ce0-60f9-4e93-8398-fa4c99e41ccc)

This folder contains all hardware and software resources for building a custom controller to operate an off-the-shelf fog machine via Arduino:

- **PCB/** – Circuit board files (Gerbers, BOM, etc.).
- **case/** – Enclosure CAD models for housing the PCB.
- **ros_service_fog_control/** – ROS services to turn the fog machine on/off from a robot.

## Requirements
- Arduino IDE (to upload `.ino` sketches)
- ROS + `rosserial` (for using the ROS services)

## Usage
1. **Assemble the PCB** – Refer to files in `PCB/`.
2. **Prepare the Case** – 3D-print or laser-cut parts from `case/`.
3. **Upload Arduino Code** – Open `ros_arduino_service.ino` in the Arduino IDE and flash it to your board.
4. **Operate** – Call the `/fog_machine/turn_on` and `/fog_machine/turn_off` ROS services (if using `ros_arduino_service.ino/`).
