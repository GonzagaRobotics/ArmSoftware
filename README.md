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

## Troubleshooting

### Troubleshooting Ethernet SSH connection

Ensure the Pi is powered on, and that the Ethernet cable is plugged into both the Pi and the laptop being used to power it. Also ensure that the ethernet cable has wires for data, and isn't just for Power over Ethernet (PoE) just in case.

Ensure ssh is installed on your laptop. Run `ssh arm@192.168.0.8` from a terminal to see if the Pi can connect over Ethernet. If that fails, you can try running `ssh arm@10.248.2.255` to connect to the Pi over WiFi to ensure the Pi is on. Note that connecting over WiFi will not enable you to use the Control System.

#### Windows:

- open Control Panel
- click on Network and Internet
- ensure Ethernet is listed there, and shows as connected
- Right click on appropriate Ethernet adapter, and select Properties (will require administrative permissions)
- Scroll down in the menu until you see IPv4 settings
- Click properties
- Change to match values shown in Network Configuration


#### Linux:
If your Desktop Environment supports it, use the proper GUI to change your network settings. The application might be labeled `Advanced Network Configuration`. If that doesn't work, do the following:

- Open a terminal
- run `ip a`. This will list all your networking devices and their statuses. Make a note of the name of your ethernet device name. It will likely be something like `eth0` or `enp2s0`. The following walkthrough will use `enp2s0`, but you should substitute that with your correct name,
- If the ethernet adapter shows as DOWN, run `sudo ip link set dev enp2s0 up`. If you don't have `sudo`, install it from your package manager.
- run `sudo ip address add 192.168.0.3/24 dev enp2s0
`
- run `sudo ip route add 192.158.0.3 via 192.168.0.254`
- You can check your routing tables to see if it's been set appropriately with `ip r`. Running `route` can also be used, but might be less clear about the gateway.
- Retry the ssh'ing into the Pi.

If these steps don't work, continue using `ip a`, `ip r`, and `route` to check the status of your device. You may need to disable some network managing services temporarily and retry these steps.
