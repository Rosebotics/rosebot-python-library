#!/usr/bin/env python3

"""
Copyright (c) 2015 Alan Yorinks All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU  General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
"""

from pymata_aio.pymata3 import PyMata3
from pymata_aio.constants import Constants
import math


class MMA8452Q:
    """
    This library is a direct port of: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library/tree/V_1.1.0
    Special Note: All reads have the Constants.I2C_END_TX_MASK bit sit. Most devices do not need to do this, but it
    is required for this chip.
    """

    def __init__(self, address, scale, output_data_rate):
        """

        @param address: Address of the device
        @param scale: scale factor
        @param output_data_rate: output data rate
        @return: no return value
        """
        self.MMA8452Q_Register = {
            'STATUS': 0x00,
            'OUT_X_MSB': 0x01,
            'OUT_Y_MSB': 0x03,
            'OUT_Y_LSB': 0x04,
            'OUT_Z_MSB': 0x05,
            'OUT_Z_LSB': 0x06,
            'SYSMOD': 0x0B,
            'INT_SOURCE': 0x0C,
            'WHO_AM_I': 0x0D,
            'XYZ_DATA_CFG': 0x0E,
            'HP_FILTER_CUTOFF': 0x0F,
            'PL_STATUS': 0x10,
            'PL_CFG': 0x11,
            'PL_COUNT': 0x12,
            'PL_BF_ZCOMP': 0x13,
            'P_L_THS_REG': 0x14,
            'FF_MT_CFG': 0x15,
            'FF_MT_SRC': 0x16,
            'FF_MT_THS': 0x17,
            'FF_MT_COUNT': 0x18,
            'TRANSIENT_CFG': 0x1D,
            'TRANSIENT_SRC': 0x1E,
            'TRANSIENT_THS': 0x1F,
            'TRANSIENT_COUNT': 0x20,
            'PULSE_CFG': 0x21,
            'PULSE_SRC': 0x22,
            'PULSE_THSX': 0x23,
            'PULSE_THSY': 0x24,
            'PULSE_THSZ': 0x25,
            'PULSE_TMLT': 0x26,
            'PULSE_LTCY': 0x27,
            'PULSE_WIND': 0x28,
            'ASLP_COUNT': 0x29,
            'CTRL_REG1': 0x2A,
            'CTRL_REG2': 0x2B,
            'CTRL_REG3': 0x2C,
            'CTRL_REG4': 0x2D,
            'CTRL_REG5': 0x2E,
            'OFF_X': 0x2F,
            'OFF_Y': 0x30,
            'OFF_Z': 0x31
        }

        # portrait landscape status values
        self.PORTRAIT_U = 0
        self.PORTRAIT_D = 1
        self.LANDSCAPE_R = 2
        self.LANDSCAPE_L = 3
        self.LOCKOUT = 0x40

        # device id
        self.device_id = 42

        # device address
        self.address = address

        # scale factor (fsr)
        self.scale = scale

        # output data rate (odr)
        self.output_data_rate = output_data_rate

        # When a read is performed,, data is returned through a call back to this structure.
        # It should be cleared after data is consumed
        self.callback_data = []

        # beginning of data returned is located at position 4
        # 0 is the device address
        self.data_start = 2

        # pymata3 is acting as a proxy for the Wire library here
        self.board = PyMata3()

        # configure firmata for i2c
        self.board.i2c_config()

        # verify the device by sending a WHO AM I command and checking the results
        if not self.check_who_am_i():
            print("Who am I fails")
            self.board.shutdown()
        else:
            # Correct device, continue with init
            # Must be in standby to change registers
            self.standby()

            # set up the scale register
            self.set_scale(self.scale)

            # set the output data rate
            self.set_output_data_rate(self.output_data_rate)

            # Set up portrait/landscape detection
            self.setup_portrait_landscape()

            # Disable x, y, set z to 0.5g
            self.setup_tap(0x80, 0x80, 0x08)

            # set device to active state
            self.board.sleep(.3)
            self.set_active()

    def data_val(self, data):
        """
        This is the callback method used to save read results
        @param data: Data returned from the device
        @return: No return value
        """
        self.callback_data = data

    def check_who_am_i(self):
        """
        This method checks verifies the device ID.
        @return: True if valid, False if not
        """
        register = self.MMA8452Q_Register['WHO_AM_I']

        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)
        reply = self.wait_for_read_result()

        if reply[self.data_start] == self.device_id:
            rval = True
        else:
            rval = False
        return rval

    def standby(self):
        """
        Put the device into standby mode so that the registers can be set.
        @return: No return value
        """
        register = self.MMA8452Q_Register['CTRL_REG1']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        ctrl1 = self.wait_for_read_result()

        ctrl1 = (ctrl1[self.data_start]) & ~0x01
        self.callback_data = []

        self.board.i2c_write_request(self.address, [register, ctrl1])

    def set_scale(self, scale):
        """
        Set the device scale register.
        Device must be in standby before calling this function
        @param scale: scale factor
        @return: No return value
        """
        register = self.MMA8452Q_Register['XYZ_DATA_CFG']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        config_reg = self.wait_for_read_result()
        config_reg = config_reg[self.data_start]
        config_reg &= 0xFC  # Mask out scale bits
        config_reg |= (scale >> 2)
        self.board.i2c_write_request(self.address, [register, config_reg])

    def set_output_data_rate(self, output_data_rate):
        """
        Set the device output data rate.
        Device must be in standby before calling this function
        @param output_data_rate: Desired data rate
        @return: No return value.
        """
        # self.standby()
        register = self.MMA8452Q_Register['CTRL_REG1']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        control_reg = self.wait_for_read_result()
        control_reg = control_reg[self.data_start]

        control_reg &= 0xC7  # Mask out data rate bits
        control_reg |= (output_data_rate << 3)
        self.board.i2c_write_request(self.address, [register, control_reg])

    def setup_portrait_landscape(self):
        """
        Setup the portrait/landscape registers
        Device must be in standby before calling this function
        @return: No return value
        """
        register = self.MMA8452Q_Register['PL_CFG']

        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        control_reg = self.wait_for_read_result()
        control_reg = control_reg[self.data_start] | 0x40

        #  1. Enable P/L
        self.board.i2c_write_request(self.address, [register, control_reg])

        register = self.MMA8452Q_Register['PL_COUNT']

        # 2. Set the de-bounce rate
        self.board.i2c_write_request(self.address, [register, 0x50])

    def read_portrait_landscape(self):
        """
        This function reads the portrait/landscape status register of the MMA8452Q.
        It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
        or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
        @return: See above.
        """
        register = self.MMA8452Q_Register['PL_STATUS']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        pl_status = self.wait_for_read_result()
        pl_status = pl_status[self.data_start]
        if pl_status & 0x40:  # Z-tilt lockout
            return self.LOCKOUT
        else:  # Otherwise return LAPO status
            return (pl_status & 0x6) >> 1

    def setup_tap(self, x_ths, y_ths, z_ths):
        """
        This method sets the tap thresholds.
        Device must be in standby before calling this function.
        Set up single and double tap - 5 steps:
        for more info check out this app note:
        http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
        Set the threshold - minimum required acceleration to cause a tap.
        @param x_ths: x tap threshold
        @param y_ths: y tap threshold
        @param z_ths: z tap threshold
        @return: No return value.
        """
        temp = 0

        if not (x_ths & 0x80):  # If top bit ISN'T set
            temp |= 0x3  # Enable taps on x
            register = self.MMA8452Q_Register["PULSE_THSX"]
            self.board.i2c_write_request(self.address, [register, x_ths])

        if not (y_ths & 0x80):  # If top bit ISN'T set
            temp |= 0x0C  # Enable taps on y
            register = self.MMA8452Q_Register["PULSE_THSY"]
            self.board.i2c_write_request(self.address, [register, y_ths])

        if not (z_ths & 0x80):  # If top bit Izx
            temp |= 0x30  # Enable taps on z
            register = self.MMA8452Q_Register["PULSE_THSZ"]
            self.board.i2c_write_request(self.address, [register, z_ths])

        # self.board.sleep(2)
        # Set up single and/or double tap detection on each axis individually.
        register = self.MMA8452Q_Register['PULSE_CFG']
        self.board.i2c_write_request(self.address, [register, temp | 0x40])

        #  Set the time limit - the maximum time that a tap can be above the thresh
        register = self.MMA8452Q_Register['PULSE_TMLT']
        #  30ms time limit at 800Hz odr
        self.board.i2c_write_request(self.address, [register, 0x30])

        #  Set the pulse latency - the minimum required time between pulses
        register = self.MMA8452Q_Register['PULSE_LTCY']
        self.board.i2c_write_request(self.address, [register, 0xA0])

        #  Set the second pulse window - maximum allowed time between end of
        #  latency and start of second pulse
        register = self.MMA8452Q_Register['PULSE_WIND']
        self.board.i2c_write_request(self.address, [register, 0xFF])  # 5. 318ms (max value) between taps max


    def read_tap(self):
        """
        This function returns any taps read by the MMA8452Q. If the function
        returns 0, no new taps were detected. Otherwise the function will return the
        lower 7 bits of the PULSE_SRC register.
        @return: 0 or lower 7 bits of the PULSE_SRC register.
        """
        register = self.MMA8452Q_Register['PULSE_SRC']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)
        #self.board.sleep(.1)
        tap_status = self.wait_for_read_result()
        tap_status = tap_status[self.data_start]
        if tap_status & 0x80:  # Read EA bit to check if a interrupt was generated
            return tap_status & 0x7F
        else:
            return 0

    def set_active(self):
        """
        This method sets the device to the active state
        @return: No return value.
        """
        register = self.MMA8452Q_Register['CTRL_REG1']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        control_reg = self.wait_for_read_result()

        control_reg = control_reg[self.data_start] | 0x01

        self.board.i2c_write_request(self.address, [register, control_reg])

    def available(self):
        """
        This method checks to see if new xyz data is available
        @return: Returns 0 if not available. 1 if it is available
        """
        register = self.MMA8452Q_Register['STATUS']
        self.board.i2c_read_request(self.address, register, 1,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        avail = self.wait_for_read_result()
        avail = (avail[self.data_start] & 0x08) >> 3

        return avail

    def read(self):
        """
        The device returns an MSB and LSB (in that order) for each axis.
        These are 12 bit values - that is only the upper 4 bits of the LSB are used.

        To make things more confusing, firmata returns each axis as 4 bytes, and reverses the order because
        it looks at the world as lsb, msb order.
        :return: callback data is set with x,y,z raw (integers) followed by x,y,z corrected ( floating point)
        Call available() first to make sure new data is really available.
        """
        register = self.MMA8452Q_Register['OUT_X_MSB']
        self.board.i2c_read_request(self.address, register, 6,
                                    Constants.I2C_READ | Constants.I2C_END_TX_MASK,
                                    self.data_val)

        # get x y z data
        xyz = self.wait_for_read_result()

        # string off address and register bytes
        xyz = xyz[2:]
        xmsb = xyz[0]
        xlsb = xyz[1]
        ymsb = xyz[2]
        ylsb = xyz[3]
        zmsb = xyz[4]
        zlsb = xyz[5]

        x = int((xmsb << 8) | xlsb) >> 4

        if xmsb > 127:
            x = 4095 - x
            x = ~x + 1

        y = int(((ymsb << 8) | ylsb)) >> 4

        if ymsb > 127:
            y = 4095 - y
            y = ~y + 1

        z = int((zmsb << 8) | zlsb) >> 4

        if zmsb > 127:
            z = 4095 - z
            z = ~z + 1

        cx = x / 2048 * self.scale
        cy = y / 2048 * self.scale
        cz = z / 2048 * self.scale

        # get portrait/landscape
        port_land = self.read_portrait_landscape()
        if port_land == self.LOCKOUT:
            port_land = 'Flat           '
        elif port_land == 0:
            port_land = 'Portrait Up    '
        elif port_land == 1:
            port_land = 'Portrait Down  '
        elif port_land == 2:
            port_land = 'Landscape Right'
        else:
            port_land = 'Landscape Left '

        # get tap status
        tap = self.read_tap()
        if tap:
            tap = "TAPPED"
        else:
            tap = "No Tap"

        angle_xz = 180 * math.atan2(x, z) / math.pi
        angle_xy = 180 * math.atan2(x, y) / math.pi
        angle_yz = 180 * math.atan2(y, z) / math.pi

        return [x, y, z, cx, cy, cz, angle_xz, angle_yz, angle_xy, port_land,
                tap]

    def wait_for_read_result(self):
        """
        This is a utility function to wait for return data call back
        @return: Returns resultant data from callback
        """
        while not self.callback_data:
            self.board.sleep(.001)
        rval = self.callback_data
        self.callback_data = []
        return rval


accel = MMA8452Q(0x1d, 2, 0)
while True:
    if accel.available():
        values = accel.read()
        print(
            '{0:4d}, {1:4d}, {2:4d}, {3:.3f}, {4:.3f}, {5:.3f},  {6:.3f}, {7:.3f}, {8:.3f}, {9:} {10:}'.format(
                values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7], values[8],
                values[9], values[10]))
    else:
        print("Where's the device?")
    accel.board.sleep(.001)
