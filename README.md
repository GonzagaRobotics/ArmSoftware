# Arm Software

Version: 0.1.0

Code name: N/A

## Introduction

Controls the robotic arm.

## Dependencies

-   System dependencies: ros-humble-rosbridge-server (if directly connecting to ControlSystem)
-   Python dependencies: pigpio

## Building

```bash
# Make sure you have the ROS2 environment sourced

# Build the packages (first time or after changes)
colcon build --symlink-install

# Source the overlay workspace (every time you open a new terminal)
source install/local_setup.bash
```

## Network Configuration

You need to configure your computer's ethernet settings to work with the arm. Have the following settings:

-   IPv4: Manual
-   Address: 192.168.0.3
-   Netmask: 255.255.255.0
-   Gateway: 192.168.0.254
-   DNS: 192.168.0.254

**Of course, your computer may just decide to not connect because it feels like it.**

## Running

Before running, make sure you call `sudo pigpiod` to start the pigpio daemon. This only needs to be done once per boot.

### Just the Arm

```bash
ros2 launch launch/launch.py
```

### The Arm directly connected to ControlSystem

Then, run the following command:

```bash
ros2 launch launch/launch_rosbridge.py
```

This will also start the rosbridge server.

See the ControlSystem README for more information on using it.
