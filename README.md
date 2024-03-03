# Arm Software

## Introduction

TL;DR: Collection of software for the robotic arm.

## Quick Start

Make sure you have the following installed on your (Ubuntu) system:

-   Modern Python 3
-   ROS2 Humble
-   System dependencies: ros-humble-rosbridge-server
-   Python dependencies: pigpio

Then, run the following commands inside the root of the repository:

```bash
# Make sure you have the ROS2 environment sourced

# Build the packages (first time only)
colcon build --symlink-install

# Source the overlay workspace (every time you open a new terminal)
source install/local_setup.bash
```

### Running just the PWM Controller

```bash
ros2 run pwm_ros pwm_ros
```

### Running the Controller using the GUI

Make sure that you have connected your machine to the Pi with an ethernet cable, and that the Pi lists the ethernet connection as "connected" in the network settings.

Then, run the following command:

```bash
ros2 launch launch/launch.py
```

See the GUI itself for further instructions on how to use it.

You might need to to change your ethernet settings on your machine to:

-   IPv4: Manual
-   Address: 192.168.0.3
-   Netmask: 255.255.255.0
-   Gateway: 192.168.0.254
-   DNS: 192.168.0.254

But try it without changing the settings first.
