import rclpy
from rclpy.node import Node, Subscription
from std_msgs.msg import Float32
import pigpio


# Define GPIO signals to use
# GPIO Pins for Step and Direction
SHOULDER_PWM = 18
SHOULDER_DIR = 17
# FOREARM_PWM = 12
# FOREARM_DIR = 6
# WRIST_PWM = 17
# WRIST_DIR = 18
FOREARM_PWM = 13
FOREARM_DIR = 26
WRIST_PWM = 16
WRIST_DIR = 6
# GPIO.setmode(GPIO.BCM)
frequency = 1000
# initialize pigpio library
pi = pigpio.pi()
pi.set_PWM_frequency(WRIST_PWM, 1000)
pi.set_PWM_dutycycle(WRIST_PWM, 0)

pi.set_mode(SHOULDER_DIR, pigpio.OUTPUT)
# GPIO.setup(SHOULDER_PWM,GPIO.OUT)
pi.set_mode(FOREARM_DIR, pigpio.OUTPUT)
# GPIO.setup(FOREARM_PWM,GPIO.OUT)
pi.set_mode(WRIST_PWM, pigpio.OUTPUT)
pi.set_mode(WRIST_DIR, pigpio.OUTPUT)

# shoulder_pwm = GPIO.PWM(SHOULDER_PWM,1000) #set freq to 1000 hz
# shoulder_pwm.start(0)
# forearm_pwm = GPIO.PWM(FOREARM_PWM,1000)
# forearm_pwm.start(0)
# shoulder_forearm = True
pi.hardware_PWM(SHOULDER_PWM, frequency, 0)
pi.hardware_PWM(FOREARM_PWM, frequency, 0)

# Definitions
FORWARD = 1
BACKWARD = 0
# Stepper Motors PINS
DIR1 = 23   # Direction GPIO Pin
STEP1 = 24  # Step GPIO Pin
ENABLE1 = 25
DIR2 = 6   # Direction GPIO Pin
STEP2 = 5  # Step GPIO Pin
ENABLE2 = 0
# Set pin as output
"""
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(ENABLE1, GPIO.OUT)
GPIO.output(ENABLE1, GPIO.HIGH)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.setup(ENABLE2, GPIO.OUT)
GPIO.output(ENABLE2, GPIO.HIGH)
"""
# period for Software PWM
period = 1.0 / 1000
duty_cycle = 0

# Set up sequence for stepping the motor
# Number of steps for one revolution depends on the motor and microstepping settings
step_count = 200
delay = 0.001  # Time between steps, in seconds, to control speed


def set_motor_speed(speed, pwm, GPIO_DIR):
    speed = int(speed * 1000000)

    if speed > 0:
        pi.write(GPIO_DIR, 0)
        pi.hardware_PWM(pwm, frequency, speed)
    elif speed < 0:
        pi.write(GPIO_DIR, 1)
        pi.hardware_PWM(pwm, frequency, -speed)
    else:
        pi.hardware_PWM(pwm, frequency, 0)


def set_motor_speed_software(speed, pwm_pin, GPIO_DIR):
    speed = int(speed * 255)

    if speed > 0:
        pi.write(GPIO_DIR, 0)
        pi.set_PWM_dutycycle(pwm_pin, speed)
    elif speed < 0:
        pi.write(GPIO_DIR, 1)
        pi.set_PWM_dutycycle(pwm_pin, -speed)
    else:
        pi.set_PWM_dutycycle(pwm_pin, 0)


class PWMROS(Node):
    shoulder_sub: Subscription
    forearm_sub: Subscription
    wrist_sub: Subscription

    def __init__(self):
        super().__init__('pwm_ros')

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
            Float32,
            '/arm/wrist',
            self.wrist_callback,
            10
        )

        self.get_logger().info('PWM ready')

    def shoulder_callback(self, msg: Float32):
        self.get_logger().info('Shoulder: %s' % msg.data)

        if (abs(msg.data) < 0.1):
            set_motor_speed(0, SHOULDER_PWM, SHOULDER_DIR)
        else:
            set_motor_speed(msg.data, SHOULDER_PWM, SHOULDER_DIR)

    def forearm_callback(self, msg: Float32):
        self.get_logger().info('Forearm: %s' % msg.data)

        if (abs(msg.data) < 0.1):
            set_motor_speed(0, FOREARM_PWM, FOREARM_DIR)
        else:
            set_motor_speed(msg.data, FOREARM_PWM, FOREARM_DIR)

    def wrist_callback(self, msg: Float32):
        self.get_logger().info('Wrist: %s' % msg.data)

        if (abs(msg.data) < 0.1):
            set_motor_speed_software(0, WRIST_PWM, WRIST_DIR)
        else:
            set_motor_speed_software(msg.data, WRIST_PWM, WRIST_DIR)


def pwm_shutdown():
    set_motor_speed(0, SHOULDER_PWM, SHOULDER_DIR)
    set_motor_speed(0, FOREARM_PWM, FOREARM_DIR)
    set_motor_speed_software(0, WRIST_PWM, WRIST_DIR)
    pi.stop()


def main(args=None):
    rclpy.init(args=args)

    pwm_ros = PWMROS()

    try:
        rclpy.spin(pwm_ros)
    finally:
        pwm_shutdown()
        pwm_ros.destroy_node()
        rclpy.shutdown()
