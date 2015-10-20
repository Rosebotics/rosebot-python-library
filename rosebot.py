#!/usr/bin/python
"""
  This class is a wrapper for the Pymata3 library that is specifically designed
  for the RoseBot.
"""

import math

from pymata_aio.constants import Constants
from pymata_aio.pymata3 import PyMata3



class RoseBotConstants:
	"""Generic constants used in the RoseBot library that don't fit well with another category."""
	# Default sleep amount
	DEFAULT_SLEEP_S = 0.025
	
	# Constants( that may or may not be included )
	INPUT = 0x00  # pin set as input
	OUTPUT = 0x01  # pin set as output
	LOW = 0x00
	HIGH = 0x01

class RoseBotPhysicalConstants:
	# Motor control pins
	PIN_LEFT_MOTOR_CONTROL_1 = 2
	PIN_LEFT_MOTOR_CONTROL_2 = 4
	PIN_LEFT_MOTOR_PWM = 5
	PIN_RIGHT_MOTOR_CONTROL_1 = 7
	PIN_RIGHT_MOTOR_CONTROL_2 = 8
	PIN_RIGHT_MOTOR_PWM = 6

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


class RoseBotConnection(PyMata3):
    """Creates the Pymata connection to the Arduino board with default parameters and returns the Pymata3 object."""
    def __init__(self, ip_address=None, com_port=None):
        if ip_address != None:
            print("Connecting to ip address " + ip_address + "...")
        elif com_port != None:
            print("Connecting via com port " + com_port + "...")
        else:
            print("Connecting via com port using automatic com port detection...")
        super().__init__(arduino_wait=0, log_output=True, com_port=com_port, ip_address=ip_address)
        self.keep_alive(2)
        print("Ready!")


class RoseBotEncoder:
	"""Track the encoder ticks.  This class should only be instantiated once and a shared reference used to
       communicate with the RoseMotor class."""
	
    shared_encoder = None # Instance of the RoseBotEncoder that is shared wit the RoseBotMotor class.
    
    def __init__(self, board):
        RoseBotEncoder.shared_encoder = self  # Save a global reference to the one and only encoder instance so that the motors can set the direction.
        self.board = board
        self.encoder_count_left = 0
        self.encoder_count_right = 0
        self.left_direction = DIRECTION_FORWARD  # default to forward if not set
        self.right_direction = DIRECTION_FORWARD  # default to forward if not set
        board.encoder_config(PIN_LEFT_ENCODER, PIN_RIGHT_ENCODER, self._encoder_callback,
                             Constants.CB_TYPE_DIRECT, True)

    def _encoder_callback(self, data):
        """Internal callback when encoder data updates."""
        if self.left_direction == DIRECTION_FORWARD:
            self.encoder_count_left += data[0]
        else:
            self.encoder_count_left -= data[0]
        if self.right_direction == DIRECTION_FORWARD:
            self.encoder_count_right += data[1]
        else:
            self.encoder_count_right -= data[1]

    def clear_enc(self, encoder_pin_to_reset=None):
        """Clears the encoder count accumulators.
           Optionally you can pass in the encoder pin value to reset only 1 of the two encoder counters."""
        if encoder_pin_to_reset == None:
            self.encoder_count_left = 0
            self.encoder_count_right = 0
        elif encoder_pin_to_reset == PIN_RIGHT_ENCODER:
            self.encoder_count_right = 0
        elif encoder_pin_to_reset == PIN_LEFT_ENCODER:
            self.encoder_count_left = 0
        
    def get_distance(self):
        """Uses the current encoder ticks and returns a value for the distance traveled in inches. Uses the minimum count of the encoders to ensure that the RoseBot is not just going around in a circle"""
        avg_encoder_count = (self.encoder_count_left + self.encoder_count_right) / 2
        return  WHEEL_CIRC_IN * avg_encoder_count / COUNTS_PER_REV
        
    
class RoseBotMotors:
    """Controls the motors on the RoseBot."""
	CLOCKWISE = 0
	COUNTER_CLOCKWISE = 1
	DEFAULT_MOTOR_SPEED = 150
	# TODO: Check if you need to use RoseBotMotor.CLOCKWISE instead of just CLOCKWISE.

	# RoseBot Wheel diameter calculations
	COUNTS_PER_REV = 192  # 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
	WHEEL_DIAM_IN = 2.56  # diam = 65mm / 25.4 mm/in WHEEL_CIRC_IN = math.pi * WHEEL_DIAM
	WHEEL_TRACK_WIDTH = 6  # approx X inches from centre of the RoseBot (pivot point) to the wheels. Used to calculate turning angles
	WHEEL_CIRC_IN = math.pi * WHEEL_DIAM_IN	
	
	DIRECTION_FORWARD = 1
	DIRECTION_REVERSE = -1


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

    def drive_pwm(self, left_pwm, right_pwm=None):
        """Drive the left and right motors using pwm values.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255).
        """
        if right_pwm is None:  # If no right_pwm is entered, then the right motor speed mirrors the left motor
            right_pwm = left_pwm
        self.drive_pwm_left(left_pwm)
        self.drive_pwm_right(right_pwm)


    def drive_pwm_left(self, pwm):
        """Drive the left motor based on the pwm value.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255)."""
        if pwm > 0:
            self._left_fwd(min(abs(pwm), 255))
        else:
            self._left_rev(min(abs(pwm), 255))

    def drive_pwm_right(self, pwm):
        """Drive the right motor based on the pwm value.
           Positive values go forwards (capped at 255).  Negative values go in reverse (capped at -255)."""
        if pwm > 0:
            self._right_fwd(min(abs(pwm), 255))
        else:
            self._right_rev(min(abs(pwm), 255))

    def drive_distance(self, distance_in, left_pwm, right_pwm=None):
        """Drives a certain distance in inches then stops with independent motor pwm values."""
		# TODO: Remove all pwm stuff.  Replace with speed_cm_per_s stuff.
        if right_pwm is None:  # If no right_speed is entered, then the right motor speed mirrors the left motor
            right_pwm = left_pwm
        left_count = 0
        right_count = 0
        num_ticks = distance_in / WHEEL_CIRC_IN * COUNTS_PER_REV
        RoseBotEncoder.shared_encoder.clear_enc()  # clear the encoder count
        self.drive_left(left_pwm)
        self.drive_right(right_pwm)
        while right_count < num_ticks  or left_count < num_ticks:
            left_count = RoseBotEncoder.shared_encoder.get_ticks(PIN_LEFT_ENCODER)
            right_count = RoseBotEncoder.shared_encoder.get_ticks(PIN_RIGHT_ENCODER)
            self.board.sleep(DEFAULT_SLEEP_S)
        self.brake()

    def turn_angle(self, angle_degrees, motor_pwm=DEFAULT_MOTOR_SPEED):
        """Turns a certain number of degrees (negative is a left turn counterclockwise, positive is a right turn clockwise)"""
        self.clear_enc()
        angle_radians = math.radians(angle_degrees)
        arc_length = WHEEL_TRACK_WIDTH / 2.0 * angle_radians
        angle_wanted_in_ticks = arc_length / WHEEL_CIRC_IN * COUNTS_PER_REV
        if angle_degrees < 0:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        else:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        while self.encoder_count_left < angle_wanted_in_ticks:
            self.board.sleep(DEFAULT_SLEEP_S)
        self.brake()
        # TODO: Test this method.  After testing delete this comment.


    # ******************************************************************************
    #  Private functions for RoseBotMotor
    # ******************************************************************************/
    def _left_fwd(self, pwm):
        """Sets the H-Bridge control lines to forward and sets the PWM value."""
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.left_direction = DIRECTION_FORWARD

    def _left_rev(self, pwm):
        """Sets the H-Bridge control lines to reverse and sets the PWM value."""
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(PIN_LEFT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.left_direction = DIRECTION_REVERSE

    def _right_fwd(self, pwm):
        """Sets the H-Bridge control lines to forward and sets the PWM value."""
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.right_direction = DIRECTION_FORWARD

    def _right_rev(self, pwm):
        """Sets the H-Bridge control lines to reverse and sets the PWM value."""
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(PIN_RIGHT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.right_direction = DIRECTION_REVERSE

class RoseBotBuzzer:
    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
            
    def play_tone(self, note, duration = None):
        self.board.play_tone(self.pin_number, Constants.TONE_TONE, note, duration)
        
    def stop(self):
        self.board.play_tone(self.pin_number, Constants.TONE_NO_TONE, 0, 0)


class RoseBotInput:
    """Gets readings from the RoseBot sensors."""

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number


class RoseBotAnalogInput(RoseBotInput):
    """Gets analog readings from the RoseBot sensors."""

    def __init__(self, board, pin_number):
        super().__init__(board, pin_number)
        board.set_pin_mode(pin_number, Constants.ANALOG)

    def read(self):
        return self.board.analog_read(self.pin_number)


class RoseBotDigitalInput(RoseBotInput):
    """Gets digital readings from the RoseBot sensors."""
    
    def __init__(self, board, pin_number):
        super().__init__(board, pin_number)
        self.board.set_pin_mode(pin_number, Constants.INPUT)
        # TODO: Change the 1 to Constants.HIGH once that constant is added.
        self.board.digital_write(pin_number, 1)  # sets pin pull-up resistor. INPUT_PULLUP is not an option with Pymata
       

    def read(self):
        return self.board.digital_read(self.pin_number)


class RoseBotServo:
    """Control servo motors connected to the RoseBot """

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
        self.board.servo_config(pin_number)
        
    def write (self, servo_position):
        self.board.analog_write(self.pin_number, servo_position)
        
        
class RoseBotDigitalOutput:
    """Control digital outputs connected to the RoseBot """
    
    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
        self.board.set_pin_mode(pin_number, Constants.OUTPUT)
        self.state = LOW 
    
    def digital_write (self, signal):
        self.board.digital_write(self.pin_number, signal)
        self.state = signal
        
    def digital_read(self):
	    """Present only if you need to reference later what you wrote to a pin."""
        return self.state

		
class RoseBotAccelerometer:
	# TODO: Implement
	pass

class RoseBotPid:
	""" 
	 This class provides a simple implementation of a pid controller for the RoseBot.
	"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, integrator_max=100, integrator_min=-100):
	    """Create a PID instance with gains provided."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.derivator = 0.0
        self.integrator = 0.0
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.set_point = 0.0
        self.error = 0.0

		
    def update(self, current_value):
        """Calculate pid output value for the given input."""
        self.error = self.set_point - current_value
        
		self.p_value = self.kp * self.error
        
		self.d_value = self.kd * (self.error - self.derivator)
        self.derivator = self.error
		
        self.integrator = self.integrator + self.error
        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min
        self.i_value = self.integrator * self.ki

        return self.p_value + self.i_value + self.d_value