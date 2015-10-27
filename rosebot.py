import math,time
from pymata_aio.constants import Constants
from pymata_aio.pymata3 import PyMata3
from .mma8452q3 import MMA8452Q3


class RoseBotConstants:
    """Generic constants used in the RoseBot library that don't fit well with another category."""
    # Rate at which Arduino board sends fresh data
    SAMPLING_INTERVAL_S = 0.025
    LOW = 0
    HIGH = 1


class RoseBotPhysicalConstants:
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
    PIN_11 = 11 # Commonly used for the right bumper or Pixy MOSI
    PIN_BUTTON = 12 # Commonly used for the button or Pixy MISO
    PIN_LED = 13

    PIN_LEFT_MOTOR_CONTROL_1 = 2
    PIN_LEFT_MOTOR_CONTROL_2 = 4
    PIN_LEFT_MOTOR_PWM = 5
    PIN_RIGHT_MOTOR_CONTROL_1 = 7
    PIN_RIGHT_MOTOR_CONTROL_2 = 8
    PIN_RIGHT_MOTOR_PWM = 6

    PIN_LEFT_ENCODER = PIN_A2_AS_DIGITAL
    PIN_RIGHT_ENCODER = PIN_10

    # RoseBot Wheel diameter calculations
    # TODO: Make our only distance unit be cm.
    COUNTS_PER_REV = 192  # 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
    WHEEL_DIAM_CM = 6.5  # diam = 65mm / 25.4 mm/in WHEEL_CIRC_CM = math.pi * WHEEL_DIAM
    WHEEL_TRACK_WIDTH_CM = 15  # approx 7.5 centimeters from centre of the RoseBot (pivot point) to the wheels. Used to calculate turning angles
    WHEEL_CIRC_CM = math.pi * WHEEL_DIAM_CM


class RoseBotConnection(PyMata3):
    """Creates the Pymata connection to the Arduino board with default parameters and returns the Pymata3 object."""
    def __init__(self, ip_address=None, com_port=None, use_log_file=True):
        reboot_time = 2
        if ip_address != None:
            print("Connecting to ip address " + ip_address + "...")
            reboot_time = 0 # Arduino does not reset if using a WiFly.
        elif com_port != None:
            print("Connecting via com port " + com_port + "...")
        else:
            print("Connecting via com port using automatic com port detection...")
        super().__init__(arduino_wait=reboot_time, log_output=use_log_file, com_port=com_port, ip_address=ip_address)
        self.keep_alive(period=2) # Send keep_alive message at 1.6 seconds, timeout at 2 seconds
        print("Ready!")


class RoseBotMotors:
    """Controls the motors on the RoseBot."""
    CLOCKWISE = 0
    COUNTER_CLOCKWISE = 1
    DEFAULT_MOTOR_SPEED = 150
    DIRECTION_FORWARD = 1
    DIRECTION_REVERSE = -1
    # PWM duty cycle = PWM_DC_PER_CM_S_SPEED_MULTIPLIER * Desired cm/s speed + PWM_OFFSET
    PWM_DC_PER_CM_S_SPEED_MULTIPLIER = 3.75 # when not usingencoder  this will give relatively accurate duty cycles
    PWM_OFFSET = 30 # this offset is due to the physical offset required to get the wheels initially driving when not
    
    def __init__(self, board):
        """Constructor for pin setup"""
        self.board = board
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_1, Constants.OUTPUT)
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_2, Constants.OUTPUT)
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_PWM, Constants.PWM)  # Choosing to set explicitly, not required
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_1, Constants.OUTPUT)
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_2, Constants.OUTPUT)
        self.board.set_pin_mode(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_PWM, Constants.PWM)  # Choosing to set explicitly, not required
        
    def brake(self):
        """Effectively shorts the two leads of the motor together, which causes the motor to resist being turned.
           Causes the robot to stop quicker."""
        self.left_brake()
        self.right_brake()

    def left_brake(self):
        """Brakes left motor, stopping it immediately"""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_PWM, 0)

    def right_brake(self):
        """Brakes right motor, stopping it immediately"""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_PWM, 0)

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

    def drive_distance(self, distance_cm, left_pwm, right_pwm=None):
        """Drives a certain distance in cm then stops with independent motor pwm values."""
        # TODO: Remove all pwm stuff.  Replace with speed_cm_per_s stuff.
        if right_pwm is None:  # If no right_speed is entered, then the right motor speed mirrors the left motor
            right_pwm = left_pwm
        num_ticks = distance_cm / RoseBotPhysicalConstants.WHEEL_CIRC_CM * RoseBotPhysicalConstants.COUNTS_PER_REV
        RoseBotEncoder.shared_encoder.reset_encoder_counts()  # clear the encoder count
        self.drive_left(left_pwm)
        self.drive_right(right_pwm)
        while RoseBotEncoder.shared_encoder.count_right < num_ticks  or \
                RoseBotEncoder.shared_encoder.count_left < num_ticks:
            self.board.sleep(RoseBotConstants.SAMPLING_INTERVAL_S)
        self.brake()        
        
    def drive_at_speed(self, speed_cm_per_s_left_motor, speed_cm_per_s_right_motor=None, use_encoder_feedback=False):
        """Function used to drive the RoseBot motors at a given speed, designated in cm/s""" 
        if speed_cm_per_s_right_motor==None:
            speed_cm_per_s_right_motor = speed_cm_per_s_left_motor
        pwm_left_motor = speed_cm_per_s_left_motor * RoseBotMotors.PWM_DC_PER_CM_S_SPEED_MULTIPLIER + RoseBotMotors.PWM_OFFSET
        pwm_right_motor = speed_cm_per_s_right_motor * RoseBotMotors.PWM_DC_PER_CM_S_SPEED_MULTIPLIER + RoseBotMotors.PWM_OFFSET
        if use_encoder_feedback: 
            if RoseBotPid.shared_pid_left == None:
                print("Initialized PID controller for left encoder feedback")
                RoseBotPid.shared_pid_left = RoseBotPid(set_point=speed_cm_per_s_left_motor)
                RoseBotEncoder.shared_encoder.reset_encoder_counts()
                RoseBotEncoder.shared_encoder.count_left_at_last_update = RoseBotEncoder.shared_encoder.count_left
            if RoseBotPid.shared_pid_right == None:
                print("Initialized PID controller for right encoder feedback")
                RoseBotPid.shared_pid_right = RoseBotPid(set_point=speed_cm_per_s_right_motor)
                RoseBotEncoder.shared_encoder.count_right_at_last_update = RoseBotEncoder.shared_encoder.count_right
        self.drive_pwm(int(pwm_left_motor), int(pwm_right_motor))
        
    def turn_angle(self, angle_degrees, motor_pwm=DEFAULT_MOTOR_SPEED):
        """Turns a certain number of degrees (negative is a left turn counterclockwise, positive is a right turn clockwise)"""
        self.reset_encoder_counts()
        angle_radians = math.radians(angle_degrees)
        arc_length = RoseBotPhysicalConstants.WHEEL_TRACK_WIDTH_CM / 2.0 * angle_radians
        angle_wanted_in_ticks = arc_length / RoseBotPhysicalConstants.WHEEL_CIRC_CM * RoseBotPhysicalConstants.COUNTS_PER_REV
        if angle_degrees < 0:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        else:
            self.motors.drive_left(motor_pwm)
            self.motors.drive_right(-motor_pwm)
        while self.count_left < angle_wanted_in_ticks:
            self.board.sleep(RoseBotConstants.SAMPLING_INTERVAL_S)
        self.brake()
        # TODO: Test this method.  After testing delete this comment.


    # ******************************************************************************
    #  Private functions for RoseBotMotor
    # ******************************************************************************/
    def _left_fwd(self, pwm):
        """Sets the H-Bridge control lines to forward and sets the PWM value."""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.left_direction = self.DIRECTION_FORWARD

    def _left_rev(self, pwm):
        """Sets the H-Bridge control lines to reverse and sets the PWM value."""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_LEFT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.left_direction = self.DIRECTION_REVERSE

    def _right_fwd(self, pwm):
        """Sets the H-Bridge control lines to forward and sets the PWM value."""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_1, 1)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_2, 0)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.right_direction = RoseBotMotors.DIRECTION_FORWARD

    def _right_rev(self, pwm):
        """Sets the H-Bridge control lines to reverse and sets the PWM value."""
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_1, 0)
        self.board.digital_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_CONTROL_2, 1)
        self.board.analog_write(RoseBotPhysicalConstants.PIN_RIGHT_MOTOR_PWM, pwm)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if RoseBotEncoder.shared_encoder:
            RoseBotEncoder.shared_encoder.right_direction = RoseBotMotors.DIRECTION_REVERSE


class RoseBotEncoder:
    """Track the encoder ticks.  This class should only be instantiated once and a shared reference used to
       communicate with the RoseMotor class."""

    shared_encoder = None # Instance of the RoseBotEncoder that is shared wit the RoseBotMotor class.

    def __init__(self, board):
        RoseBotEncoder.shared_encoder = self  # Save a global reference to the one and only encoder instance so that the motors can set the direction.
        self.board = board
        self.count_left = 0
        self.count_right = 0
        self.count_left_at_last_update = 0
        self.count_right_at_last_update = 0
        self.update_shared_pids_on_encoder_callback = False
        self.time_of_last_update = time.perf_counter()        
        self.left_direction = RoseBotMotors.DIRECTION_FORWARD  # default to forward if not set
        self.right_direction = RoseBotMotors.DIRECTION_FORWARD  # default to forward if not set
        board.encoder_config(RoseBotPhysicalConstants.PIN_LEFT_ENCODER, RoseBotPhysicalConstants.PIN_RIGHT_ENCODER, self._encoder_callback,
                             Constants.CB_TYPE_DIRECT, True)

    def _encoder_callback(self, data):
        """Internal callback when encoder data updates."""
        if self.left_direction == RoseBotMotors.DIRECTION_FORWARD:
            self.count_left += data[0]
        else:
            self.count_left -= data[0]
        if self.right_direction == RoseBotMotors.DIRECTION_FORWARD:
            self.count_right += data[1]
        else:
            self.count_right -= data[1]
        if self.update_shared_pids_on_encoder_callback:
            # TODO: Implement the encoder feedback to actually update the wheel pwms.
            RoseBotPid.shared_pid_left.update()
            RoseBotPid.shared_pid_left.update()

    def reset_encoder_counts(self):
        """Clears the encoder count accumulators.
           Optionally you can pass in the encoder pin value to reset only 1 of the two encoder counters."""
        self.count_left = 0
        self.count_right = 0

    def get_distance(self):
        """Uses the current encoder ticks and returns a value for the distance traveled in centimeters. Uses the minimum count of the encoders to ensure that the RoseBot is not just going around in a circle"""
        avg_encoder_count = (self.count_left + self.count_right) / 2
        return RoseBotPhysicalConstants.WHEEL_CIRC_CM * avg_encoder_count / RoseBotPhysicalConstants.COUNTS_PER_REV

    def get_angle(self):
        """Get angle. Uses a simple calculation that shows which direction the RoseBot is currently pointing. Note: It DOES NOT 
        compute the angle at which is has been drving at, only the relative orientation of the RoseBot compared to how is was originally orientated
        TODO: Work out how to do that accurately"""
        diff_in_encoder_counts = self.count_left - self.count_right
        relative_arc_length_traveled = diff_in_encoder_counts * RoseBotPhysicalConstants.WHEEL_CIRC_CM
        angle_in_radians = relative_arc_length_traveled / (RoseBotPhysicalConstants.WHEEL_TRACK_WIDTH_CM / 2)
        return math.degrees(angle_in_radians)
        
    def update_speed(self):
        # TODO: Get this working. Having problems with the speed updates being able to keep up.
        current_time = time.perf_counter
        delta_sec = current_time - self.time_of_last_update
        self.speed_left = (self.count_left - self.count_left_at_last_update) / delta_sec
        self.speed_right = (self.count_right - self.count_right_at_last_update) / delta_sec
        self.time_of_last_update = current_time
        self.count_left_at_last_update = self.count_left
        self.count_right_at_last_update = self.count_right
        
    
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


class RoseBotBuzzer:
    #TODO: Add some notes (approx 12-20 notes)

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number

    def play_tone(self, note, duration_s=None):
        duration_ms = None
        if duration_s != None:
            duration_ms = int(duration_s * 1000) 
        self.board.play_tone(self.pin_number, Constants.TONE_TONE, note, duration_ms)

    def stop(self):
        self.board.play_tone(self.pin_number, Constants.TONE_NO_TONE, 0)


class RoseBotDigitalOutput:
    """Control digital outputs connected to the RoseBot """

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
        self.board.set_pin_mode(pin_number, Constants.OUTPUT)
        self.state = RoseBotConstants.LOW

    def digital_write (self, signal):
        self.board.digital_write(self.pin_number, signal)
        self.state = signal
        

    def digital_read(self):
        """Present only if you need to reference later what you wrote to a pin."""
        return self.state

class RoseBotPid:
    """
     This class provides a simple implementation of a pid controller for the RoseBot.
    """
    shared_pid_left = None  # shared reference to the PID controller for the left motor
    shared_pid_right = None  # shared reference to the PID controller for the left motor

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, integrator_max=100, integrator_min=-100, set_point=0.0):
        """Create a PID instance with gains provided."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.derivator = 0.0
        self.integrator = 0.0
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.set_point = set_point
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


class RoseBotAccelerometer(MMA8452Q3):
    """An import of the mma8452q3 library"""
    
    # Positions in the returned array for the accel.read() function
    VAL_RAW_X = 0
    VAL_RAW_Y = 1
    VAL_RAW_Z = 2
    VAL_X = 3  # Corrected X Value. The CX,CY,CZ Values take the 0-2048 raw value and convert them to a 'G-value' 
    VAL_Y = 4
    VAL_Z = 5
    VAL_ANGLE_XZ = 6
    VAL_ANGLE_YZ = 7
    VAL_ANGLE_XY = 8
    VAL_PORT_LAND = 9
    VAL_TAPPED = 10
    
    
    
    def __init__(self, board):
        DEVICE_ADDRESS = 0x1d  # Physical board address of the accelerometer 
        SCALE = 2  # sets a scaling factor for the outputted results
        OUTPUT_DATA_RATE = 0
    
        super().__init__(board, DEVICE_ADDRESS, SCALE, OUTPUT_DATA_RATE)
        self.board = board

