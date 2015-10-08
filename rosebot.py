#!/usr/bin/python
"""
  This class is a partial port from the https://github.com/sparkfun/RedBot
  Arduino library into Pymata-aio, with added features for use with the RoseBot
  
"""
from pymata_aio.constants import Constants
from rosebot_development.robot.rosebot_pid import PID
import math

# Constants( that may or may not be included )

PI = 3.14
CLOCKWISE = 0
COUNTER_CLOCKWISE = 1
DEFAULT_MOTOR_SPEED = 150

# RoseBot Wheel diameter calculations
COUNT_PER_REV = 192  # 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
WHEEL_DIAM = 2.56  # diam = 65mm / 25.4 mm/in WHEEL_CIRC = math.pi * WHEEL_DIAM
WHEEL_SPREAD = 3  # approx X inches from centre of the RedBot (pivot point) to the wheels. Used to calculate turning angles
WHEEL_CIRC = PI * WHEEL_DIAM
WHEEL_CIRC__DIV_COUNT_PER_REV = PI * WHEEL_DIAM * COUNT_PER_REV



# RedBot motor pins from RedBot.h
L_CTRL_1 = 2
L_CTRL_2 = 4
PWM_L = 5

R_CTRL_1 = 7
R_CTRL_2 = 8
PWM_R = 6

ENCODER_PIN_LEFT = 16
ENCODER_PIN_RIGHT = 10

DIRECTION_FORWARD = 1
DIRECTION_REVERSE = -1

PIN_LEFT_LINE_FOLLOWER = 3
PIN_CENTER_LINE_FOLLOWER = 6
PIN_RIGHT_LINE_FOLLOWER = 7
PIN_A0 = 0
PIN_A1 = 1
PIN_A2 = 16
PIN_A3 = 17
PIN_A4 = 4
PIN_A5 = 5
PIN_A6 = 6
PIN_A7 = 7
PIN_3 = 3
PIN_9 = 9
PIN_10 = 10
PIN_11 = 11
PIN_BUTTON = 12
PIN_LED = 13

encoder_object = None
pid_controller = PID(1, 0, 0)  # TODO: Ask if this variable should/could be accessed from within example scripts?

class RedBotEncoder:
    def __init__(self, board):
        global encoder_object
        encoder_object = self  # Save a global reference to the one and only encoder_object so that the motors can set the direction.
        self.board = board
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_direction = DIRECTION_FORWARD  # default to forward if not set
        self.right_direction = DIRECTION_FORWARD  # default to forward if not set
        board.encoder_config(ENCODER_PIN_LEFT, ENCODER_PIN_RIGHT, self.encoder_callback,
                             Constants.CB_TYPE_DIRECT, True)

    def encoder_callback(self, data):
        if self.left_direction == DIRECTION_FORWARD:
            self.left_encoder_count += data[0]
        else:
            self.left_encoder_count -= data[0]
        if self.right_direction == DIRECTION_FORWARD:
            self.right_encoder_count += data[1]
        else:
            self.right_encoder_count -= data[1]

    def clear_enc(self, flag=None):
        if flag == None:
            self.left_encoder_count = 0
            self.right_encoder_count = 0
        elif flag == ENCODER_PIN_RIGHT:
            self.right_encoder_count = 0
        elif flag == ENCODER_PIN_LEFT:
            self.left_encoder_count = 0

    def get_ticks(self, encoder):
        if encoder == ENCODER_PIN_LEFT:
            return self.left_encoder_count
        elif encoder == ENCODER_PIN_RIGHT:
            return self.right_encoder_count

    def drive_distance(self, distance, motor_power):
        left_count = 0
        right_count = 0
        num_rev = distance / WHEEL_CIRC

        print("drive_distance() {} inches at {} power for {:.2f} revolutions".format(distance, motor_power, num_rev))

        self.encoders.clear_enc()  # clear the encoder count
        self.motors.drive(motor_power)

        while right_count < self.num_rev * COUNT_PER_REV:
            left_count = self.encoders.get_ticks(ENCODER_PIN_LEFT)
            right_count = self.encoders.get_ticks(ENCODER_PIN_RIGHT)
            print("{}       {}       stop once over {:.0f} ticks".format(left_count, right_count,
                                                                         num_rev * COUNT_PER_REV))
            self.board.sleep(0.1)

        self.motors.brake()

    def get_turn_angle(self):

        total_revolutions = self.left_encoder_count / COUNT_PER_REV
        angle_in_radians = (WHEEL_DIAM * PI) * total_revolutions / WHEEL_SPREAD  # this is calculating the arc angle
        angle_in_degrees = angle_in_radians * 360 / (2 * PI)

        return angle_in_degrees

    def turn_angle(self, angle_wanted, direction, motor_speed=DEFAULT_MOTOR_SPEED):
        self.clear_enc()
        angle_wanted_in_ticks = WHEEL_CIRC * self.get_ticks(self.left_encoder_count) / COUNT_PER_REV
        angle_turned_in_ticks = 0


        while angle_turned_in_ticks < angle_wanted_in_ticks:

            if direction == CLOCKWISE:
                self.motors.drive(motor_speed)

            elif direction == COUNTER_CLOCKWISE:
                self.motors.drive(-motor_speed)  # TODO: Fix this to drive at current turning speed
                total_revolutions = self.left_encoder_count / COUNT_PER_REV
                angle_in_radians = (WHEEL_CIRC) * total_revolutions / WHEEL_SPREAD  # this is calculating the arc angle
                angle_in_degrees = angle_in_radians * 360 / (2 * PI)







class RedBotMotors:
    """Controls the motors on the RedBot"""

    def __init__(self, board):
        """Constructor for pin setup"""

        self.board = board
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        # The interface to the motor driver is kind of ugly. It's three pins per
        # channel: two that define role (forward, reverse, stop, brake) and one
        # PWM input for speed.
        board.set_pin_mode(L_CTRL_1, Constants.OUTPUT)
        board.set_pin_mode(L_CTRL_2, Constants.OUTPUT)
        board.set_pin_mode(PWM_L, Constants.PWM)  # Not done in RedBot motors but I just went ahead and added it.
        board.set_pin_mode(R_CTRL_1, Constants.OUTPUT)
        board.set_pin_mode(R_CTRL_2, Constants.OUTPUT)
        board.set_pin_mode(PWM_R, Constants.PWM)  # Not done in RedBot motors but I just went ahead and added it.

    def brake(self):
        """effectively shorts the two leads of the motor together, which causes the motor to resist being turned. It stops quite quickly."""
        self.left_brake()
        self.right_brake()

    def drive(self, left_speed, right_speed=None, durationS=-1.0):
        """
            Starts both motors. It figures out whether the motors should go
            forward or reverse, then calls the appropriate individual functions. Note
            the use of a 16-bit integer for the speed input an 8-bit integer doesn't
            have the range to reach full speed. The calls to the actual drive functions
            are only 8-bit, since we only have 8-bit PWM.
        """

        if right_speed is None:  # If no right_speed is entered, then the right motor speed mirrors the left motor
            right_speed = left_speed

        if left_speed > 0:
            self.left_fwd(min(abs(left_speed), 255))
        else:
            self.left_rev(min(abs(left_speed), 255))
        if right_speed > 0:
            self.right_fwd(min(abs(right_speed), 255))
        else:
            self.right_rev(min(abs(right_speed), 255))

        if durationS > 0:
            self.board.sleep(durationS)
            self.left_stop()
            self.right_stop()

    def drive_speed(self, desired_speed, encoder_left, encoder_right, use_encoder_feedback=False):


        if encoder_object:
        # TODO: finsh/rework converting encoder ticks to speeds

            current_left_speed = encoder_left.get_ticks() * WHEEL_CIRC__DIV_COUNT_PER_REV  # all constants lumped together into one -> pi*wheel_diam/ticks_per_revolution
            current_right_speed = encoder_right.get_ticks() * WHEEL_CIRC__DIV_COUNT_PER_REV
            left_motor_power = pid_controller.update(desired_speed - current_left_speed)
            right_motor_power = pid_controller.update(desired_speed - current_right_speed)
            self.left_fwd(left_motor_power)
            self.drive(left_motor_power, right_motor_power)

        else:
            # guess speeds #TODO: MAP PWMS TO SPEEDS
            pass


    def left_motor(self, speed, durationS=-1.0):
        """Basically the same as drive(), but omitting the right motor."""
        if speed > 0:
            self.left_rev(min(abs(speed), 255))
        else:
            self.left_fwd(min(abs(speed), 255))
        if durationS > 0:
            self.board.sleep(durationS)
            self.left_stop()

    def right_motor(self, speed, durationS=-1.0):
        """Basically the same as drive(), but omitting the left motor."""
        if speed > 0:
            self.right_fwd(min(abs(speed), 255))
        else:
            self.right_rev(min(abs(speed), 255))
        if durationS > 0:
            self.board.sleep(durationS)
            self.left_stop()

    def stop(self):
        """
            stop() allows the motors to coast to a stop, rather than trying to stop them
            quickly. As will be the case with functions affecting both motors, the
            global stop just calls the individual stop functions for each wheel.
        """
        self.left_stop()
        self.right_stop()

    def left_brake(self):
        """brakes left motor, stopping it immediately"""
        self.board.digital_write(L_CTRL_1, 1)
        self.board.digital_write(L_CTRL_2, 1)
        self.board.analog_write(PWM_L, 0)

    def right_brake(self):
        """brakes right motor, stopping it immediately"""
        self.board.digital_write(R_CTRL_1, 1)
        self.board.digital_write(R_CTRL_2, 1)
        self.board.analog_write(PWM_R, 0)

    def left_stop(self):
        """allows left motor to coast to a stop"""
        self.board.digital_write(L_CTRL_1, 0)
        self.board.digital_write(L_CTRL_2, 0)
        self.board.analog_write(PWM_L, 0)

    def right_stop(self):
        """allows left motor to coast to a stop"""
        self.board.digital_write(R_CTRL_1, 0)
        self.board.digital_write(R_CTRL_2, 0)
        self.board.analog_write(PWM_R, 0)

    def pivot(self, speed, durationS=-1.0):
        """
            pivot() controls the pivot speed of the RedBot. The values of the pivot function inputs
            range from -255:255, with -255 indicating a full speed counter-clockwise rotation.
            255 indicates a full speed clockwise rotation
        """
        if speed < 0:
            self.left_fwd(min(abs(speed), 255))
            self.right_rev(min(abs(speed), 255))
        else:
            self.left_rev(min(abs(speed), 255))
            self.right_fwd(min(abs(speed), 255))
        if durationS > 0:
            self.board.sleep(durationS)
            self.left_stop()
            self.right_stop()


    # ******************************************************************************
    #  Private functions for RedBotMotor
    # ******************************************************************************/
    # These are the motor-driver level abstractions for turning a given motor the
    #  right direction. Users never see them, and *should* never see them, so we
    #  make them private.

    def left_fwd(self, spd):
        self.board.digital_write(L_CTRL_1, 1)
        self.board.digital_write(L_CTRL_2, 0)
        self.board.analog_write(PWM_L, spd)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.left_direction = DIRECTION_FORWARD

    def left_rev(self, spd):
        self.board.digital_write(L_CTRL_1, 0)
        self.board.digital_write(L_CTRL_2, 1)
        self.board.analog_write(PWM_L, spd)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.left_direction = DIRECTION_REVERSE

    def right_fwd(self, spd):
        self.board.digital_write(R_CTRL_1, 1)
        self.board.digital_write(R_CTRL_2, 0)
        self.board.analog_write(PWM_R, spd)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            encoder_object.right_direction = DIRECTION_FORWARD

    def right_rev(self, spd):
        self.board.digital_write(R_CTRL_1, 0)
        self.board.digital_write(R_CTRL_2, 1)
        self.board.analog_write(PWM_R, spd)
        # If we have an encoder in the system, we want to make sure that it counts
        # in the right direction when ticks occur.
        if encoder_object:
            print("Right is in reverse")
            encoder_object.right_direction = DIRECTION_REVERSE


class RedBotSensor:
    pin_number = 0

    def __init__(self, board, pin_number):
        self.board = board
        self.pin_number = pin_number
        board.set_pin_mode(pin_number, Constants.ANALOG)

    def read(self):
        return self.board.analog_read(self.pin_number)


class RedBotBumper:
    pin_number = 0

    def __init__(self, board, pin_number):
        self.pin_number = pin_number
        self.board = board
        self.board.set_pin_mode(pin_number, Constants.INPUT)
        self.board.digital_write(pin_number, 1)  # sets pin pull-up resistor. INPUT_PULLUP is not an option with Pymata
        pass

    def read(self):
        return self.board.digital_read(self.pin_number)
