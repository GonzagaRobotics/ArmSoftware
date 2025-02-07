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

# Build the packages
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

To run the arm, run `ros2 run arm arm`. This will start the arm node.

The node has 6 optional parameters:

-   shoulder_pwm: GPIO pin for the shoulder PWM signal
-   shoulder_dir: GPIO pin for the shoulder direction signal
-   forearm_pwm: GPIO pin for the forearm PWM signal
-   forearm_dir: GPIO pin for the forearm direction signal
-   wrist_pwm: GPIO pin for the wrist PWM signal
-   wrist_dir: GPIO pin for the wrist direction signal

To set these parameters run `ros2 run arm arm --ros-args -p [param_name]:=[value]`.

Example:

```bash
ros2 run arm arm --ros-args -p shoulder_pwm:=17 -p shoulder_dir:=19 -p forearm_dir:=1
```

## Controlling Directly

If you want to run the arm with the ControlSystem, open a new terminal and run `ros2 launch rosbridge_server rosbridge_websocket.launch.xml` to start the rosbridge server.
