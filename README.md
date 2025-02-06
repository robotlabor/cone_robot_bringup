# Bringup guide for Cone Robot

## Overview
This guide provides instructions for launching ROS 2 nodes to integrate an ODrive motor driver via CAN and a CubePilot CubeOrange controller on the Cone Robot platform. 

## Prerequisites
Ensure the following prerequisites are met before executing the commands:

- ROS 2 Humble is installed.
- The `robot_ws` workspace is built and sourced.
- The CubePilot CubeOrange is connected via USB and accessible through `/dev/serial/by-id/`.
- The ODrive CAN setup is configured correctly.

## Launching the drive interface
To start the drive interface, open a terminal and execute the following commands:

```bash
source /opt/ros/humble/setup.bash
source robot_ws/install/setup.bash
ros2 launch odrive_can launch4.yaml
```

This package has been set up to start everyting required for driving the platform via the /cmd_vel ROS 2 topic. CAN interfaces are started automatically as the on-board computer is powered.

## Launching MAVROS for CubePilot CubeOrange
In a second terminal, start MAVROS with the CubeOrange flight controller:

```bash
source /opt/ros/humble/setup.bash
source robot_ws/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00
```

## Notes
- Ensure that the appropriate ROS 2 environment is sourced before launching any nodes.
- Verify that the device path (`/dev/serial/by-id/...`) corresponds to the actual connected CubeOrange device.
- Check for any permission issues that may require `udev` rules or `chmod` adjustments for accessing serial devices.
- CAN interfaces are started automatically via services. The manual equivalent for troubleshooting can be found below:
- 1. Enable the USB CAN device:
   ```bash
   sudo slcand -o -s6 -t hw -S 3000000 /dev/ttyACM2 can0
   ```
2. Activate the CAN network:
   ```bash
   sudo ip link set can0 up type can bitrate 250000
   ```


## Troubleshooting
If any issues arise during execution, consider the following debugging steps:

- Use `ros2 topic list` to verify active topics.
- Check `ros2 node list` to ensure the expected nodes are running.
- Inspect logs using `ros2 param get /mavros fcu_url` and `ros2 param list` for configuration details.
- Ensure CAN communication with ODrive is functional using `candump` or other debugging tools.


