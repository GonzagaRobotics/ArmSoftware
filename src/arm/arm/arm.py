import rclpy
from rclpy.node import Node, Subscription
from std_msgs.msg import Float32, Int32
import pigpio
import time

# Direction definitions
FORWARD = 1
BACKWARD = 0

# Linear actuator pins
SHOULDER_PWM = 13
SHOULDER_DIR = 6
FOREARM_PWM = 19
FOREARM_DIR = 26
WRIST_PWM = 12
WRIST_DIR = 16

# Base motor pins
BASE_PWM = 22
BASE_DIR = 27

# Linear actuator PWM frequency
ACTUATOR_FREQUENCY = 1000

# Stepper motors pins
MINOR_X_DIR = 14
MINOR_X_STEP = 15

# Number of steps per revolution
STEPPER_SPR = 200
# Time between each step (seconds)
STEPPER_TICK_DELAY = 0.001

# Last time the step pins were toggled
LAST_STEPPER_TICK_TIME = 0
# Last value of the step pin
MINOR_X_LAST_VALUE = 0
# Are we currently stepping?
MINOR_X_ON = 0

PI: pigpio.pi


def set_motor_speed(speed, pwm, GPIO_DIR):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 1000000)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, -speed)
    else:
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, 0)


def set_motor_speed_software(speed, pwm_pin, GPIO_DIR):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 255)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.set_PWM_dutycycle(pwm_pin, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.set_PWM_dutycycle(pwm_pin, -speed)
    else:
        PI.set_PWM_dutycycle(pwm_pin, 0)


def set_minor_x(dir: int):
    global MINOR_X_ON

    MINOR_X_ON = dir != 0
    PI.write(MINOR_X_DIR, dir > 0)


def stepper_tick():
    global LAST_STEPPER_TICK_TIME
    global MINOR_X_LAST_VALUE

    if time.time() - LAST_STEPPER_TICK_TIME < STEPPER_TICK_DELAY:
        return

    LAST_STEPPER_TICK_TIME = time.time()

    if MINOR_X_ON:
        MINOR_X_LAST_VALUE = not MINOR_X_LAST_VALUE
        PI.write(MINOR_X_STEP, MINOR_X_LAST_VALUE)
    elif MINOR_X_LAST_VALUE:
        PI.write(MINOR_X_STEP, 0)
        MINOR_X_LAST_VALUE = 0


def start():
    global PI

    PI = pigpio.pi()

    # PI.set_PWM_frequency(WRIST_PWM, 1000)
    # PI.set_PWM_dutycycle(WRIST_PWM, 0)
    PI.set_PWM_frequency(FOREARM_PWM, 1000)
    PI.set_PWM_dutycycle(FOREARM_PWM, 0)

    PI.set_mode(BASE_PWM, pigpio.OUTPUT)
    PI.set_mode(BASE_DIR, pigpio.OUTPUT)
    PI.set_mode(SHOULDER_PWM, pigpio.OUTPUT)
    PI.set_mode(SHOULDER_DIR, pigpio.OUTPUT)
    PI.set_mode(FOREARM_PWM, pigpio.OUTPUT)
    PI.set_mode(FOREARM_DIR, pigpio.OUTPUT)
    PI.set_mode(WRIST_PWM, pigpio.OUTPUT)
    PI.set_mode(WRIST_DIR, pigpio.OUTPUT)
    PI.set_mode(MINOR_X_STEP, pigpio.OUTPUT)
    PI.set_mode(MINOR_X_DIR, pigpio.OUTPUT)

    PI.hardware_PWM(SHOULDER_PWM, ACTUATOR_FREQUENCY, 0)
    # PI.hardware_PWM(FOREARM_PWM, ACTUATOR_FREQUENCY, 0)
    PI.hardware_PWM(WRIST_PWM, ACTUATOR_FREQUENCY, 0)


def shutdown():
    set_motor_speed(0, SHOULDER_PWM, SHOULDER_DIR)
    # set_motor_speed(0, FOREARM_PWM, FOREARM_DIR)
    set_motor_speed_software(0, FOREARM_PWM, FOREARM_DIR)
    # set_motor_speed_software(0, WRIST_PWM, WRIST_DIR)
    set_motor_speed(0, WRIST_PWM, WRIST_DIR)
    set_motor_speed_software(0, BASE_PWM, BASE_DIR)
    set_minor_x(0)

    PI.stop()


class Arm(Node):
    base_sub: Subscription
    shoulder_sub: Subscription
    forearm_sub: Subscription
    wrist_sub: Subscription
    minor_x_sub: Subscription

    def __init__(self):
        super().__init__('arm')

        # Declare parameters, with the default values being the current global values
        # Any parameters set at runtime will override these

        global SHOULDER_PWM, SHOULDER_DIR, FOREARM_PWM, FOREARM_DIR, WRIST_PWM, WRIST_DIR

        SHOULDER_PWM = self.declare_parameter(
            'shoulder_pwm', SHOULDER_PWM).value
        SHOULDER_DIR = self.declare_parameter(
            'shoulder_dir', SHOULDER_DIR).value
        FOREARM_PWM = self.declare_parameter(
            'forearm_pwm', FOREARM_PWM).value
        FOREARM_DIR = self.declare_parameter(
            'forearm_dir', FOREARM_DIR).value
        WRIST_PWM = self.declare_parameter(
            'wrist_pwm', WRIST_PWM).value
        WRIST_DIR = self.declare_parameter(
            'wrist_dir', WRIST_DIR).value

        self.base_sub = self.create_subscription(
            Float32,
            '/arm/base',
            self.base_callback,
            10
        )

        self.shoulder_sub = self.create_subscription(
            Float32,
            '/arm/shoulder',
            self.shoulder_callback,
            10
        )

        self.forearm_sub = self.create_subscription(
            Float32,
            '/arm/forearm',
            self.forearm_callback,
            10
        )

        self.wrist_sub = self.create_subscription(
            Int32,
            '/arm/wrist',
            self.wrist_callback,
            10
        )

        self.minor_x_sub = self.create_subscription(
            Int32,
            '/arm/minor/x',
            self.minor_x_callback,
            10
        )

    def base_callback(self, msg: Float32):
        self.get_logger().info('Base: %s' % msg.data)

        set_motor_speed_software(msg.data, BASE_PWM, BASE_DIR)

    def shoulder_callback(self, msg: Float32):
        self.get_logger().info('Shoulder: %s' % msg.data)

        set_motor_speed(msg.data, SHOULDER_PWM, SHOULDER_DIR)

    def forearm_callback(self, msg: Float32):
        self.get_logger().info('Forearm: %s' % msg.data)

        # set_motor_speed(msg.data, FOREARM_PWM, FOREARM_DIR)
        set_motor_speed_software(msg.data, FOREARM_PWM, FOREARM_DIR)

    def wrist_callback(self, msg: Int32):
        self.get_logger().info('Wrist: %s' % msg.data)

        # set_motor_speed_software(msg.data, WRIST_PWM, WRIST_DIR)
        set_motor_speed(msg.data, WRIST_PWM, WRIST_DIR)

    def minor_x_callback(self, msg: Int32):
        self.get_logger().info('Minor X: %s' % msg.data)

        set_minor_x(msg.data)


def main(args=None):
    rclpy.init(args=args)

    arm = Arm()

    start()

    arm.get_logger().info('Arm ready')

    try:
        while rclpy.ok():
            rclpy.spin_once(arm, timeout_sec=0)
            stepper_tick()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
