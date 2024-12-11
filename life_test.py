import pigpio
import time

# Direction definitions
FORWARD = 1
BACKWARD = 0

# Stepper motors pins
STEPPER_DIR = 23
STEPPER_STEP = 24
STEPPER_ENABLE = 25

# Number of steps per revolution
STEPPER_STEP_COUNT = 200
# Time between each step (seconds)
STEPPER_TICK_DELAY = 0.001

# Last time the step pins were toggled
LAST_STEPPER_TICK_TIME = 0

# Last value of the step pin
STEPPER_LAST_VALUE = 0

PI: pigpio.pi


def move_stepper(steps, direction):
    global PI
    global LAST_STEPPER_TICK_TIME
    global STEPPER_LAST_VALUE

    PI.write(STEPPER_DIR, direction)

    steps_taken = 0

    while steps_taken < steps * 2:
        if time.time() - LAST_STEPPER_TICK_TIME < STEPPER_TICK_DELAY:
            continue

        PI.write(STEPPER_STEP, 1 - STEPPER_LAST_VALUE)
        STEPPER_LAST_VALUE = 1 - STEPPER_LAST_VALUE

        steps_taken += 1
        LAST_STEPPER_TICK_TIME = time.time()


def main():
    global PI

    PI = pigpio.pi()

    PI.set_mode(STEPPER_DIR, pigpio.OUTPUT)
    PI.set_mode(STEPPER_STEP, pigpio.OUTPUT)
    PI.set_mode(STEPPER_ENABLE, pigpio.OUTPUT)

    PI.write(STEPPER_ENABLE, 1)

    move_stepper(200, FORWARD)

    time.sleep(1)

    move_stepper(200, BACKWARD)

    PI.write(STEPPER_ENABLE, 0)

    PI.stop()


if __name__ == '__main__':
    main()
