#!/usr/bin/python
"""
  This class is a partial port from the https://github.com/sparkfun/RoseBot
  Arduino library into Pymata-aio, with added features for use with the RoseBot

"""
from pymata_aio.constants import Constants
import math

# Default sleep amount
DEFAULT_SLEEP_S = 0.025

# Constants( that may or may not be included )
CLOCKWISE = 0
COUNTER_CLOCKWISE = 1
DEFAULT_MOTOR_SPEED = 150

# RoseBot Wheel diameter calculations
COUNT_PER_REV = 192  # 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
WHEEL_DIAM_IN = 2.56  # diam = 65mm / 25.4 mm/in WHEEL_CIRC_IN = math.pi * WHEEL_DIAM
WHEEL_TRACK_WIDTH = 3  # approx X inches from centre of the RoseBot (pivot point) to the wheels. Used to calculate turning angles
WHEEL_CIRC_IN = math.pi * WHEEL_DIAM_IN

# Motor control pins
PIN_LEFT_MOTOR_CONTROL_1 = 2
PIN_LEFT_MOTOR_CONTROL_2 = 4
PIN_LEFT_MOTOR_PWM = 5
PIN_RIGHT_MOTOR_CONTROL_1 = 7
PIN_RIGHT_MOTOR_CONTROL_2 = 8
PIN_RIGHT_MOTOR_PWM = 6

DIRECTION_FORWARD = 1
DIRECTION_REVERSE = -1

# Sensor pins
PIN_A0 = 0
PIN_A0_AS_DIGITAL = 14 # Also used for SoftwareSerial Tx
PIN_A1 = 1
PIN_A1_AS_DIGITAL = 15 # Also used for SoftwareSerial Rx
PIN_A2 = 2
PIN_A2_AS_DIGITAL = 16 # Commonly used for the left encoder
PIN_A3 = 3 # Commonly used for left line or distance sensor
PIN_A3_AS_DIGITAL = 17
PIN_A4 = 4
PIN_A4_AS_DIGITAL = 18 # Also used for I2C
PIN_A5 = 5
PIN_A5_AS_DIGITAL = 19 # Also used for I2C
PIN_A6 = 6 # Commonly used for center line or distance sensor
PIN_A6_AS_DIGITAL = 20
PIN_A7 = 7 # Commonly used for right line or distance sensor
PIN_A7_AS_DIGITAL = 21
PIN_3 = 3 # Common used for the left bumper
PIN_9 = 9 # Common location for the buzzer
PIN_10 = 10 # Commonly used for the right encoder
PIN_11 = 11 # Commonly used for the right bumper
PIN_BUTTON = 12
PIN_LED = 13

# Encoders
PIN_LEFT_ENCODER = PIN_A2_AS_DIGITAL
PIN_RIGHT_ENCODER = PIN_10


encoder_object = None

class RoseBotEncoder:
    """Track the encoder ticks.  This class should only be instantiated once and a global reference used to
       communicate with the RoseMotor class."""
    def __init__(self, board):
        global encoder_object
        encoder_object = self  # Save a global reference to the one and only encoder_object so that the motors can set the direction.
        self.board = board
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_direction = DIRECTION_FORWARD  # default to forward if not set
        self.right_direction = DIRECTION_FORWARD  # default to forward if not set
        board.encoder_config(PIN_LEFT_ENCODER, PIN_RIGHT_ENCODER, self._encoder_callback,
                             Constants.CB_TYPE_DIRECT, True)

    def _encoder_callback(self, data):
        """Internal callback when encoder data updates."""
        if self.left_direction == DIRECTION_FORWARD:
            self.left_encoder_count += data[0]
        else:
            self.left_encoder_count -= data[0]
        if self.right_direction == DIRECTION_FORWARD:
            self.right_encoder_count += data[1]
        else:
            self.right_encoder_count -= data[1]

    def clear_enc(self, encoder_pin_to_reset=None):
        """Clears the encoder count accumulators.
           Optionally you can pass in the encoder pin value to reset only 1 of the two encoder counters."""
        if encoder_pin_to_reset == None:
            self.left_encoder_count = 0
            self.right_encoder_count = 0
        elif encoder_pin_to_reset == PIN_RIGHT_ENCODER:
            self.right_encoder_count = 0
        elif encoder_pin_to_reset == PIN_LEFT_ENCODER:
            self.left_encoder_count = 0

    def get_ticks(self, encoder_pin):
        """Pass in the encoder pin value to get the number of ticks for that encoder."""
        if encoder_pin == PIN_LEFT_ENCODER:
            return self.left_encoder_count
        elif encoder_pin == PIN_RIGHT_ENCODER:
            return self.right_encoder_count


class RoseBotMotors:
    """Controls the motors on the RoseBot."""

    def __init__(self, board):
        """Constructor for pin setup"""
        self.board = board
        board.set_pin_mode(PIN_LEFT_MOTOR_CONTROL_1, Constants.OUTPUT)
        board.set_pin_mode(PIN_LEFT_MOTOR_CONTROL_2, Constants.OUTPUT)
        board.set_pin_mode(PIN_LEFT_MOTOR_PWM, Constants.PWM)  # Choosing to set explicitly, not required
        board.set_pin_mode(PIN_RIGHT_MOTOR_CONTROL_1, Constants.OUTPUT)
        board.set_pin_mode(PIN_RIGHT_MOTOR_CONTROL_2, Constants.OUTPUT)
        board.set_pin_mode(PIN_RIGHT_MOTOR_PWM, Constants.PWM)  # Choosing to set explicitly, not required

    def brake(self):
        """Effectively shorts the two leads of the motor together, which causes the motor to resist being turned.
           Causes the robot to stop quicker."""
        self.left_brake()
        self.right_brake()

    def left_brake(self):
        """Brakes left motor, stopping it immediately"""
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_LEFT_MOTOR_PWM, 0)

    def right_brake(self):
        """Brakes right motor, stopping it immediately"""
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_RIGHT_MOTOR_PWM, 0)

    def drive(self, left_pwm, right_pwm=None):
        """Drive the left and right motors using pwm values.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255).
        """
        if right_pwm is None:  # If no right_pwm is entered, then the right motor speed mirrors the left motor
            right_pwm = left_pwm
        self.drive_left(left_pwm)
        self.drive_right(right_pwm)


    def drive_left(self, pwm):
        """Drive the left motor based on the pwm value.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255)."""
        if pwm > 0:
            self._left_fwd(min(abs(pwm), 255))
        else:
            self._left_rev(min(abs(pwm), 255))

    def drive_right(self, pwm):
        """Drive the right motor based on the pwm value.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255)."""
        if pwm > 0:
            self._right_fwd(min(abs(pwm), 255))
        else:
            self._right_rev(min(abs(pwm), 255))

    def drive_distance(self, distance_in, left_pwm, right_pwm=None):
        """Drives a certain distance in inches then stops with independent motor pwm values."""
        if right_pwm is None:  # If no right_speed is entered, then the right motor speed mirrors the left motor
            right_pwm = left_pwm
        left_count = 0
        right_count = 0
        num_ticks = distance_in / WHEEL_CIRC_IN * COUNT_PER_REV
        encoder_object.clear_enc()  # clear the encoder count
        self.drive_left(left_pwm)
        self.drive_right(right_pwm)
        while right_count < num_ticks  or left_count < num_ticks:
            left_count = self.encoders.get_ticks(PIN_LEFT_ENCODER)
            right_count = self.encoders.get_ticks(PIN_RIGHT_ENCODER)
            self.board.sleep(DEFAULT_SLEEP_S)
        self.brake()

    def turn_angle(self, angle_degrees, motor_pwm=DEFAULT_MOTOR_SPEED):
        """Turns a certain number of degrees (negative is a left turn counterclockwise, positive is a right turn clockwise)"""
        self.clear_enc()
        angle_radians = math.radians(angle_degrees)
        arc_length = WHEEL_TRACK_WIDTH / 2.0 * angle_radians
        angle_wanted_in_ticks = arc_length / WHEEL_CIRC_IN * COUNT_PER_REV
        if angle_degrees < 0:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        else:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        while self.left_encoder_count < angle_wanted_in_ticks:
            self.board.sleep(DEFAULT_SLEEP_S)
        self.brake()
        # TODO: Test this method.  After testing delete this comment.

    # ******************************************************************************
    #  Private functions for RoseBotMotor
    # ******************************************************************************/
    def _left_fwd(self, pwm):
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.left_direction = DIRECTION_FORWARD

    def _left_rev(self, pwm):
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.left_direction = DIRECTION_REVERSE

    def _right_fwd(self, pwm):
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.right_direction = DIRECTION_FORWARD

    def _right_rev(self, pwm):
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.right_direction = DIRECTION_REVERSE


class RoseBotSensor:
    """Gets readings from the RoseBot sensors."""
    pin_number = 0

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number

class RoseBotAnalogSensor(RoseBotSensor):
    """Gets analog readings from the RoseBot sensors."""

    def __init__(self, board, pin_number):
        super().__init__(board, pin_number)
        board.set_pin_mode(pin_number, Constants.ANALOG)

    def read(self):
        return self.board.analog_read(self.pin_number)


class RoseBotDigitalSensor(RoseBotSensor):
    """Gets digital readings from the RoseBot sensors."""
    def __init__(self, board, pin_number):
        super().__init__(board, pin_number)
        self.board.set_pin_mode(pin_number, Constants.INPUT)
        # TODO: Change the 1 to Constants.HIGH once that constant is added.
        self.board.digital_write(pin_number, 1)  # sets pin pull-up resistor. INPUT_PULLUP is not an option with Pymata
        pass

    def read(self):
        return self.board.digital_read(self.pin_number)

