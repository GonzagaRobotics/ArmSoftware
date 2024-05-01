import rclpy
from rclpy.node import Node, Subscription
from std_msgs.msg import Float32, Int32
import pigpio
import time

# Direction definitions
FORWARD = 1
BACKWARD = 0

# Linear actuator pins
SHOULDER_PWM = 18
SHOULDER_DIR = 17
FOREARM_PWM = 13
FOREARM_DIR = 26
WRIST_PWM = 16
WRIST_DIR = 6

# Base motor pins
BASE_PWM = 15
BASE_DIR = 14

# Linear actuator PWM frequency
FREQUENCY = 1000

# Stepper motors pins
STEPPER_X_DIR = 2
STEPPER_X = 3
STEPPER_X_ENABLE = 4
STEPPER_Z_DIR = 12
STEPPER_Z = 5
STEPPER_Z_ENABLE = 19

# Number of steps per revolution
STEPPER_STEP_COUNT = 200
# Time between each step (seconds)
STEPPER_TICK_DELAY = 0.001

# Last time the step pins were toggled
LAST_STEPPER_TICK_TIME = 0
# Last value of the step pin
STEPPER_X_LAST_VALUE = 0
STEPPER_Z_LAST_VALUE = 0
# Are we currently stepping?
STEPPER_X_ON = 0
STEPPER_Z_ON = 0

# Pin for the grabber servo
GRABBER = 27

PI: pigpio.pi


def set_motor_speed(speed, pwm, GPIO_DIR):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 1000000)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.hardware_PWM(pwm, FREQUENCY, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.hardware_PWM(pwm, FREQUENCY, -speed)
    else:
        PI.hardware_PWM(pwm, FREQUENCY, 0)


def set_motor_speed_software(speed, pwm_pin, GPIO_DIR):
    if abs(speed) < 0.1:
        PI.set_PWM_dutycycle(pwm_pin, 0)
        return

    speed = int(speed * 255)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.set_PWM_dutycycle(pwm_pin, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.set_PWM_dutycycle(pwm_pin, -speed)


def set_servo_pos(pwm_pin, pos):
    # Limit the position to 500-2500
    pos = max(500, min(2500, pos))

    PI.set_servo_pulsewidth(pwm_pin, pos)


def move_servo(pwm_pin, direction):
    pos = PI.get_servo_pulsewidth(pwm_pin)

    if direction == FORWARD:
        pos += 100
    else:
        pos -= 100

    set_servo_pos(pwm_pin, pos)


def set_stepper_speed(motor: str, dir: int):
    # Allow us to change the global variables
    global STEPPER_X_ON
    global STEPPER_Z_ON

    if motor == 'x':
        STEPPER_X_ON = dir != 0
        PI.write(STEPPER_X_DIR, dir > 0)
        PI.write(STEPPER_X_ENABLE, not STEPPER_X_ON)
    elif motor == 'z':
        STEPPER_Z_ON = dir != 0
        PI.write(STEPPER_Z_DIR, dir > 0)
        PI.write(STEPPER_Z_ENABLE, not STEPPER_Z_ON)


def stepper_tick():
    global LAST_STEPPER_TICK_TIME
    global STEPPER_X_LAST_VALUE
    global STEPPER_Z_LAST_VALUE

    if time.time() - LAST_STEPPER_TICK_TIME < STEPPER_TICK_DELAY:
        return

    LAST_STEPPER_TICK_TIME = time.time()

    if STEPPER_X_ON:
        STEPPER_X_LAST_VALUE = not STEPPER_X_LAST_VALUE
        PI.write(STEPPER_X, STEPPER_X_LAST_VALUE)
    elif STEPPER_X_LAST_VALUE:
        PI.write(STEPPER_X, 0)
        STEPPER_X_LAST_VALUE = 0

    if STEPPER_Z_ON:
        STEPPER_Z_LAST_VALUE = not STEPPER_Z_LAST_VALUE
        PI.write(STEPPER_Z, STEPPER_Z_LAST_VALUE)
    elif STEPPER_Z_LAST_VALUE:
        PI.write(STEPPER_Z, 0)
        STEPPER_Z_LAST_VALUE = 0


def shutdown():
    set_motor_speed(0, SHOULDER_PWM, SHOULDER_DIR)
    set_motor_speed(0, FOREARM_PWM, FOREARM_DIR)
    set_motor_speed_software(0, WRIST_PWM, WRIST_DIR)
    set_stepper_speed('x', 0)
    set_stepper_speed('z', 0)

    PI.stop()


class PWMROS(Node):
    base_sub: Subscription
    shoulder_sub: Subscription
    forearm_sub: Subscription
    wrist_sub: Subscription
    minor_x_sub: Subscription
    minor_z_sub: Subscription
    minor_grab_sub: Subscription

    def __init__(self):
        super().__init__('pwm_ros')

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

        self.minor_z_sub = self.create_subscription(
            Int32,
            '/arm/minor/z',
            self.minor_z_callback,
            10
        )

        # Move Grabber to the initial position
        # set_servo_pos(GRABBER, 1500)

        self.minor_grab_sub = self.create_subscription(
            Int32,
            '/arm/minor/grab',
            self.minor_grab_callback,
            10
        )

        self.get_logger().info('Arm ready')

    def base_callback(self, msg: Float32):
        self.get_logger().info('Base: %s' % msg.data)

        set_motor_speed_software(msg.data, BASE_PWM, BASE_DIR)

    def shoulder_callback(self, msg: Float32):
        self.get_logger().info('Shoulder: %s' % msg.data)

        set_motor_speed(msg.data, SHOULDER_PWM, SHOULDER_DIR)

    def forearm_callback(self, msg: Float32):
        self.get_logger().info('Forearm: %s' % msg.data)

        set_motor_speed(msg.data, FOREARM_PWM, FOREARM_DIR)

    def wrist_callback(self, msg: Int32):
        self.get_logger().info('Wrist: %s' % msg.data)

        set_motor_speed_software(msg.data, WRIST_PWM, WRIST_DIR)

    def minor_x_callback(self, msg: Int32):
        self.get_logger().info('Minor X: %s' % msg.data)

        set_stepper_speed('x', msg.data)

    def minor_z_callback(self, msg: Int32):
        self.get_logger().info('Minor Z: %s' % msg.data)

        set_stepper_speed('z', msg.data)

    def minor_grab_callback(self, msg: Int32):
        self.get_logger().info('Minor Grab: %s' % msg.data)

        # move_servo(GRABBER, msg.data > 0)


def main(args=None):
    global PI

    PI = pigpio.pi()

    PI.set_PWM_frequency(WRIST_PWM, 1000)
    PI.set_PWM_dutycycle(WRIST_PWM, 0)

    PI.set_mode(BASE_PWM, pigpio.OUTPUT)
    PI.set_mode(BASE_DIR, pigpio.OUTPUT)
    PI.set_mode(SHOULDER_PWM, pigpio.OUTPUT)
    PI.set_mode(SHOULDER_DIR, pigpio.OUTPUT)
    PI.set_mode(FOREARM_DIR, pigpio.OUTPUT)
    PI.set_mode(WRIST_PWM, pigpio.OUTPUT)
    PI.set_mode(WRIST_DIR, pigpio.OUTPUT)

    PI.hardware_PWM(SHOULDER_PWM, FREQUENCY, 0)
    PI.hardware_PWM(FOREARM_PWM, FREQUENCY, 0)

    rclpy.init(args=args)

    pwm_ros = PWMROS()

    try:
        while rclpy.ok():
            rclpy.spin_once(pwm_ros, timeout_sec=0)
            stepper_tick()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
