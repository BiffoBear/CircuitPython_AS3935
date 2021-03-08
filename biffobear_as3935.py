# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT
"""
`biffobear_as3935`
================================================================================

CircuitPython driver library for the AS3935 lightning detector over SPI or I2C
buses.

.. warning:: The AS3935 chip supports I2C but Sparkfun found it unreliable.

* Author(s): Martin Stephens

Implementation Notes
--------------------

**Hardware:**

* A lightning detector board based on the Franklin AS3935 IC.

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit's Bus bus library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
from collections import namedtuple
from micropython import const
import digitalio
import adafruit_bus_device.spi_device as spi_dev
import adafruit_bus_device.i2c_device as i2c_dev


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/BiffoBear/Biffobear_CircuitPython_AS3935.git"

# Data structure for storing the sensor register details
_Register = namedtuple("Register", ["addr", "offset", "mask"])

# Global buffer for bus data and address
_BUFFER = bytearray(2)


def _reg_value_from_choices(value, choices):
    """Index of a value."""
    # Returns the index of a value in an iterable
    try:
        return choices.index(value)
    except ValueError as error:
        raise ValueError(
            "Select a value from %s" % ", ".join([str(x) for x in choices])
        ) from error


def _value_is_in_range(value, *, lo_limit, hi_limit):
    """Check value is in range."""
    try:
        assert lo_limit <= value <= hi_limit
    except AssertionError as error:
        raise ValueError(
            "Value must be in the range %s to %s, inclusive." % (lo_limit, hi_limit)
        ) from error
    return value


class AS3935_Sensor:
    """Register handling for the Franklin AS3935 SPI and I2C drivers."""

    # Constants to reduce RAM usage
    _0X00 = const(0x00)
    _0X01 = const(0x01)
    _0X02 = const(0x02)
    _0X03 = const(0x03)
    _0X04 = const(0x04)
    _0X05 = const(0x05)
    _0X06 = const(0x06)
    _0X07 = const(0x07)
    _0X08 = const(0x08)
    _0X0B = const(0x0B)
    _0X0F = const(0x0F)
    _0X10 = const(0x10)
    _0X12 = const(0x12)
    _0X20 = const(0x20)
    _0X3F = const(0x3F)
    _0X40 = const(0x40)
    _0XC0 = const(0xC0)
    _0XFF = const(0xFF)
    # Constants to make register values human readable in the code
    DATA_PURGE = _0X00  # Distance recalculated after purging old data
    NOISE = _0X01  # INT_NH Noise level too high. Stays high while noise remains
    DISTURBER = _0X04  # INT_D  Disturber detected
    LIGHTNING = _0X08  # INT_L  Lightning strike detected
    DIRECT_COMMAND = const(0x96)

    # Valid inputs for strike count threshold and frequency divisor registers
    _LIGHTNING_COUNT = (_0X01, _0X05, const(0x09), _0X10)
    _FREQ_DIVISOR = (_0X10, _0X20, _0X40, const(0x80))

    # AS3935 registers:

    # DISP_FLAGS combines DISP_LCO, DISP_SRCO and DISP_TRCO into a 3 bit register
    # DISP_LCO - Display antenna frequency to interrupt pin
    # DISP_SRCO - Display SRCO clock frequency to interrupt pin
    # DISP_TRCO - Display TRCO clock frequency to interrupt pin
    # SCRO_CALIB and TRCO_CALIB combine the XXXX-CALIB_DONE and XXXX-CALIB-NOK regs into 2 bit regs
    # XXXX_CALIB_DONE - Calibration completed successfully
    # XXXX_CALIB_NOK - Calibration completed unsuccessfully

    # _REGISTER_NAME = _Register(address, offset, mask)
    _PWD = _Register(_0X00, _0X00, _0X01)  # Sensor power down state
    _AFE_GB = _Register(_0X00, _0X01, const(0x3E))  # AFE gain boost
    _WDTH = _Register(_0X01, _0X00, _0X0F)  # Watchdog threshold
    _NF_LEV = _Register(_0X01, _0X04, const(0x70))  # Noise floor level
    _SREJ = _Register(_0X02, _0X00, _0X0F)  # Spike rejection
    _MIN_NUM_LIGH = _Register(_0X02, _0X04, const(0x30))  # Minimum number of lightning
    _CL_STAT = _Register(_0X02, _0X06, _0X40)  # Clear statistics
    _INT = _Register(_0X03, _0X00, _0X0F)  # Interrupt
    _MASK_DIST = _Register(_0X03, _0X05, _0X20)  # Mask disturber
    _LCO_FDIV = _Register(_0X03, _0X06, _0XC0)  # Frequency divisor for antenna tuning
    _S_LIG_L = _Register(_0X04, _0X00, _0XFF)  # Energy of single lightning LSBYTE
    _S_LIG_M = _Register(_0X05, _0X00, _0XFF)  # Energy of single lightning MSBYTE
    _S_LIG_MM = _Register(
        _0X06, _0X00, const(0x1F)
    )  # Energy of single lightning MMSBYTE
    _DISTANCE = _Register(_0X07, _0X00, _0X3F)  # Distance estimation
    _TUN_CAP = _Register(_0X08, _0X00, _0X0F)  # Internal tuning capacitance
    _DISP_FLAGS = _Register(
        _0X08, _0X05, const(0xE0)
    )  # Display flags for output to interrupt pin
    _TRCO_CALIB = _Register(const(0x3A), _0X06, _0XC0)  # TRCO calibration result
    _SRCO_CALIB = _Register((0x3B), _0X06, _0XC0)  # SRCO calibration result
    _PRESET_DEFAULT = _Register(
        const(0x3C), _0X00, _0XFF
    )  # Set this to 0x96 to reset the sensor
    _CALIB_RCO = _Register(
        const(0x3D), _0X00, _0XFF
    )  # Set this to 0x96 to calibrate the clocks

    def __init__(self, *, interrupt_pin):
        self._interrupt_pin = digitalio.DigitalInOut(interrupt_pin)
        self._interrupt_pin.direction = digitalio.Direction.INPUT
        self._startup_checks()

    def _read_byte_in(self, register):
        """Read one byte from the selected address."""
        # Stub method for testing. Overridden when subclass instatiates class.

    def _write_byte_out(self, register, data):
        """Write one byte to the selected register."""
        # Stub method for testing. Overridden when subclass instatiates class.

    def _get_register(self, register):
        """Read the current register byte, mask and shift the value."""
        return (self._read_byte_in(register) & register.mask) >> register.offset

    def _set_register(self, register, value):
        """Read the byte containing the register, mask in the new value and write out the byte."""
        # pylint: disable=assignment-from-no-return
        register_byte = self._read_byte_in(register)
        # pylint: enable=assignment-from-no-return
        register_byte &= ~register.mask
        register_byte |= (value << register.offset) & self._0XFF
        self._write_byte_out(register, register_byte)

    @property
    def indoor(self):
        """bool: Get or set Indoor mode. This must be set to True if the sensor is used indoors.
        and False if the sensor is used outdoors.  Default is True.
        """
        # Register _AFE_GB is set to self._0X12 for Indoor mode and 0x0e for outdoor mode
        if self._get_register(self._AFE_GB) == self._0X12:
            return True
        return False

    @indoor.setter
    def indoor(self, value):
        assert isinstance(value, bool)
        if value:
            self._set_register(self._AFE_GB, self._0X12)
        else:
            self._set_register(self._AFE_GB, const(0x0E))

    @property
    def watchdog(self):
        """int: Watchdog threshold in the range 0 - 10 (default is 2). Higher thresholds reduce
        triggers from disturbers but decrease sensitivity to lightning strikes.
        """
        return self._get_register(self._WDTH)

    @watchdog.setter
    def watchdog(self, value):
        self._set_register(
            self._WDTH, _value_is_in_range(value, lo_limit=0, hi_limit=10)
        )

    @property
    def noise_floor_limit(self):
        """int: Get or set the noise floor limit threshold in the range 0 - 7 (default is 2). When
        this threshold is exceeded, an interrupt is issued. Higher values allow operation with
        higher background noise but decrease sensitivity to lightning strikes.
        """
        return self._get_register(self._NF_LEV)

    @noise_floor_limit.setter
    def noise_floor_limit(self, value):
        self._set_register(
            self._NF_LEV, _value_is_in_range(value, lo_limit=0, hi_limit=7)
        )

    @property
    def spike_threshold(self):
        """int: Get or set the spike rejection threshold in the range 0 - 11 (default is 2). Higher
        values reduce false triggers but decrease sensitivity to lightning strikes.
        """
        return self._get_register(self._SREJ)

    @spike_threshold.setter
    def spike_threshold(self, value):
        self._set_register(
            self._SREJ, _value_is_in_range(value, lo_limit=0, hi_limit=11)
        )

    @property
    def energy(self):
        """int: The calculated energy of the last lightning strike. This is a
        dimensionless number.
        """
        mmsb = self._get_register(self._S_LIG_MM)
        msb = self._get_register(self._S_LIG_M)
        lsb = self._get_register(self._S_LIG_L)
        return ((mmsb << self._0X10) | (msb << self._0X08) | lsb) & const(0x3FFFFF)

    @property
    def distance(self):
        """int: Estimated distance to the storm front (km). Returns None if storm front is out of
        range (> 40 km).
        """
        distance = self._get_register(self._DISTANCE)
        if distance == self._0X3F:  # Storm out of range
            distance = None
        elif distance == self._0X01:  # Storm overhead
            distance = self._0X00
        return distance  # Distance in km

    @property
    def interrupt_status(self):
        """int: Status of the interrupt register. These constants are defined as helpers:
        LIGHTNING, DISTURBER. NOISE, DATA_PURGE.

        Note: This register is automatically cleared by the sensor after it is read.
        """
        # Wait a minimum of 2 ms between the interrupt pin going high and reading the register
        time.sleep(0.0002)
        return self._get_register(self._INT)

    @property
    def disturber_mask(self):
        """bool: Disturber mask. If the mask is True, disturber events do not
        cause interrupts. Default is False.
        """
        return bool(self._get_register(self._MASK_DIST))

    @disturber_mask.setter
    def disturber_mask(self, value):
        # Set the register value to self._0X01 to suppress disturber event interrupts
        # Set the register value to self._0X00 to allow disturber event interrupts
        assert isinstance(value, bool)
        self._set_register(self._MASK_DIST, int(value))

    @property
    def strike_count_threshold(self):
        """int: Lightning strike count threshold. The minimum number of
        lightning events before the interrupt is triggered. This threshold is
        reset to the default value of 1 after being triggered.

        Threshold may be 1, 5, 9, or 16. Default is 1.
        """
        # Convert the register value to the threshold
        return self._LIGHTNING_COUNT[self._get_register(self._MIN_NUM_LIGH)]

    @strike_count_threshold.setter
    def strike_count_threshold(self, value):
        self._set_register(
            self._MIN_NUM_LIGH, _reg_value_from_choices(value, self._LIGHTNING_COUNT)
        )

    def clear_stats(self):
        """Clear statistics from lightning distance emulation block. This resets the
        data used to calculate the distance to the storm front.
        """
        self._set_register(self._CL_STAT, self._0X01)
        self._set_register(self._CL_STAT, self._0X00)
        self._set_register(self._CL_STAT, self._0X01)

    @property
    def power_down(self):
        """bool: Power status. If True, the unit is powered off although the SPI and I2C buses
        remain active."""
        return bool(self._get_register(self._PWD))

    @power_down.setter
    def power_down(self, value):
        # If True, power down the chip. Don't check current state because it doesn't matter
        # whether the chip is already powered down
        # If False check the current state. If the chip is already powered up, do nothing
        # otherwise, power up the chip then calibrate and check the clocks
        assert isinstance(value, bool)  # Be specific because of Python's truthiness
        if value:
            self._set_register(self._PWD, self._0X01)
        elif self.power_down:
            # Only do this if the power_down mode is already set as clocks get calibrated
            self._set_register(self._PWD, self._0X00)
            # RCO clocks need to be calibrated when powering back up from a power_down
            # Procedure as per AS3935 datasheet
            self.calibrate_clocks()
            self._check_clock_calibration()
            self._set_register(self._DISP_FLAGS, self._0X02)
            time.sleep(0.002)
            self._set_register(self._DISP_FLAGS, self._0X00)

    @property
    def freq_divisor(self):
        """int: Antenna frequency divisor. The antenna resonant frequency is divided by
        this value whenever it is output as a square wave on the interrupt pin.

        Value must be one of 16, 32, 64, or 128. Default is 16.
        """
        # Convert the register value to the divisor
        return self._FREQ_DIVISOR[self._get_register(self._LCO_FDIV)]

    @freq_divisor.setter
    def freq_divisor(self, value):
        self._set_register(
            self._LCO_FDIV, _reg_value_from_choices(value, self._FREQ_DIVISOR)
        )

    @property
    def output_antenna_freq(self):
        """bool: When True, the antenna resonant frequency is divided by the freq_divisor
        and output as a square wave on the interrupt pin. Default is False.
        """
        return self._get_register(self._DISP_FLAGS) == 4

    @output_antenna_freq.setter
    def output_antenna_freq(self, value):
        assert isinstance(value, bool)
        # Set the register value to self._0X04 to enable antenna tuning mode
        # Set the register value to self._0X00 to disable antenna tuning mode
        self._set_register(self._DISP_FLAGS, int(value) << 2)

    @property
    def output_srco(self):
        """bool: When True, output the SRCO clock signal on the interrupt pin.
        Default is False.
        """
        return self._get_register(self._DISP_FLAGS) == 2

    @output_srco.setter
    def output_srco(self, value):
        assert isinstance(value, bool)
        # Set the register value to self._0X02 to output SRCO clock to the interrupt pin
        # Set the register value to self._0X00 to allow normal interrupt operation
        self._set_register(
            self._DISP_FLAGS, int(value) << 1
        )  # True is self._0X02, False is self._0X00

    @property
    def output_trco(self):
        """bool: When True, output the TRCO clock signal on the interrupt pin.
        Default is False.
        """
        return self._get_register(self._DISP_FLAGS) == 1

    @output_trco.setter
    def output_trco(self, value):
        assert isinstance(value, bool)
        # Set the register value to self._0X01 to output SRCO clock to the interrupt pin
        # Set the register value to self._0X00 to allow normal interrupt operation
        self._set_register(
            self._DISP_FLAGS, int(value)
        )  # True is self._0X01, False is self._0X00

    @property
    def tuning_capacitance(self):
        """int: The tuning capacitance for the RLC antenna in pF. This capacitance
        is added to the antenna to tune it to within 3.5 % of 500 kHz (483 - 517 kHz).

        Capacitance must be in the range 0 - 120. Any of these values may be used,
        however, the capacitance is set in steps of 8 pF, so values less than 120
        will be rounded down to the nearest step. Default is 0.
        """
        return self._get_register(self._TUN_CAP) * 8

    @tuning_capacitance.setter
    def tuning_capacitance(self, value):
        self._set_register(
            self._TUN_CAP, _value_is_in_range(value, lo_limit=0, hi_limit=120) // 8
        )

    def _check_clock_calibration(self):
        """Check clock calibration was successful."""
        # trco_result and srco_result are self._0X00 until the calibration is complete
        # For each clock, the respective register is set to self._0X01 for failure and to self._0X02
        # for success
        # Use a timeout in case the caliration register is never set (e.g. due to no comms
        # with the sensor)
        start = time.monotonic()
        trco_result, srco_result = self._0X00, self._0X00
        while not (trco_result and srco_result):
            if time.monotonic() - start > self._0X01:
                raise OSError(
                    "Problem communicating with the sensor. Check your wiring."
                )
            trco_result = self._get_register(self._TRCO_CALIB)
            srco_result = self._get_register(self._SRCO_CALIB)
        if self._0X01 in [trco_result, srco_result]:
            raise RuntimeError("AS3935 RCO clock calibration failed.")

    def calibrate_clocks(self):
        """Recalibrate the internal clocks. The clocks rely on the tuning frequency of
        the antenna, so adjust that to 500 KHz +/- 3.5 % before calibrating."""
        # Send the direct command to the CALIB_RCO register to start automatic RCO calibration
        # then check that the calibration has succeeded
        self._set_register(self._CALIB_RCO, self.DIRECT_COMMAND)
        self._check_clock_calibration()

    def reset(self):
        """Reset all the settings to the manufacturer's defaults."""
        # Send the direct command to the PRESET_DEFAUTLT register to start reset settings
        self._set_register(self._PRESET_DEFAULT, self.DIRECT_COMMAND)

    @property
    def interrupt_set(self):
        """bool: The state of the interrupt pin. Returns True if the pin is high,
        False if the pin is low and None if the pin is set to output a clock or
        antenna frequency. The pin is pulled low again after the interrupt Status
        register is read. If the the resgister is not read, then the pin is pulled
        low again as follows: After 1.0 second for a lightning event, after 1.5 seconds
        for a disturber event. The interrupt pin is held high for the duration of high noise.
        """
        # Return None if the interrupt pin is set to output a clock or antenna frequency,
        # otherwise, return the state of the interrupt pin
        if self._get_register(self._DISP_FLAGS):
            return None
        return self._interrupt_pin.value

    def _startup_checks(self):
        """Check communication with the AS3935 and confirm clocks are calibrated."""
        # With no sensor connected, reading the SPI bus returns self._0X00. After a reset
        # the clocks are calibrated automatically. Therefore, resetting the sensor then
        # checking the clock calibration status tells the that the clocks are OK and if
        # the calibration times out, we know that there are no comms with the sensor
        self.reset()
        self.calibrate_clocks()
        self._check_clock_calibration()


class AS3935_I2C(AS3935_Sensor):
    """Driver for the Franklin AS3935 with an I2C connection.

    :param busio.I2C i2c: The I2C bus connected to the chip.
    :param int address: The I2C address of the chip. Default is self._0X03.
    :param ~board.Pin interrupt_pin: The pin connected to the chip's interrupt line. Note
        that CircuitPython currently does not support interrupts, but the line is held high
        for at least one second per event, so it may be polled. Some single board computers,
        e.g. the Raspberry Pi, do support interrupts.
    """

    def __init__(self, i2c, address=const(0x03), *, interrupt_pin):
        self._bus = i2c_dev.I2CDevice(i2c, address)
        super().__init__(interrupt_pin=interrupt_pin)

    def _write_byte_out(self, register, data):
        """Write one byte to the selected register."""
        # Overrides AS3935._write_byte_out to handle writing data to the I2C bus
        # AS3935 chip returns unexpected 0x00s intermittently
        # Short pause to space out consecutive calls
        time.sleep(0.01)
        _BUFFER[0] = register.addr
        _BUFFER[1] = data
        with self._bus as bus:
            bus.write(_BUFFER, end=2)

    def _read_byte_in(self, register):
        """Read one byte from the selected register."""
        # Overrides AS3935._read_byte_in to handle writing data to the I2C bus
        # AS3935 chip returns unexpected 0x00s intermittently
        # Short pause to space out consecutive calls
        time.sleep(0.01)
        _BUFFER[0] = register.addr
        with self._bus as bus:
            bus.write_then_readinto(_BUFFER, _BUFFER, out_end=1, in_end=1)
        return _BUFFER[0]


class AS3935(AS3935_Sensor):
    """Driver for the Franklin AS3935 with a SPI connection.

    :param busio.SPI spi: The SPI bus connected to the chip.  Ensure SCK, MOSI, and MISO are
        connected.
    :param ~board.Pin cs: The pin connected to the chip's CS/chip select line.
    :param int baudrate: SPI bus baudrate. Defaults to 1,000,000 . If another baudrate is
        selected, avoid +/- 500,000 as this may interfere with the chip's antenna.
    :param ~board.Pin interrupt_pin: The pin connected to the chip's interrupt line. Note
        that CircuitPython currently does not support interrupts, but the line is held high
        for at least one second per event, so it may be polled. Some single board computers,
        e.g. the Raspberry Pi, do support interrupts.
    """

    def __init__(self, spi, cs_pin, baudrate=1_000_000, *, interrupt_pin):
        self._bus = spi_dev.SPIDevice(
            spi, digitalio.DigitalInOut(cs_pin), baudrate=baudrate, polarity=1, phase=0
        )
        super().__init__(interrupt_pin=interrupt_pin)

    def _write_byte_out(self, register, data):
        """Write one byte to the selected register."""
        # AS3935 chip returns unexpected 0x00s intermittently
        # Short pause to space out consecutive calls
        time.sleep(0.01)
        _BUFFER[0] = register.addr & self._0X3F  # Set bits 15 and 14 to 00 - write
        _BUFFER[1] = data
        with self._bus as bus:
            bus.write(_BUFFER, end=2)

    def _read_byte_in(self, register):
        """Read one byte from the selected address."""
        # AS3935 chip returns unexpected 0x00s intermittently
        # Short pause to space out consecutive calls
        time.sleep(0.01)
        _BUFFER[0] = (register.addr & const(0x3F)) | const(
            0x40
        )  # Set bits 15 and 14 to 01 - read
        with self._bus as bus:
            bus.write(_BUFFER, end=1)
            bus.readinto(_BUFFER, end=1)
            return _BUFFER[0]
