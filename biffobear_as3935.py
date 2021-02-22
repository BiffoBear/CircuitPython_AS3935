# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT
"""
`biffobear_as3935`
================================================================================

CircuitPython driver library for the AS3935 lightning detector.

.. warning:: The AS3935 chip supports I2C but Sparkfun found it unreliable so
   this driver is SPI only.


* Author(s): Martin Stephens

Implementation Notes
--------------------

**Hardware:**

* Lightning detector board based on the Franklin AS3935 IC.

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
from collections import namedtuple
from micropython import const
import digitalio
import adafruit_bus_device.spi_device as spi_dev


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/BiffoBear/Biffobear_CircuitPython_AS3935.git"

# Data structure for storing the sensor register details.
_Register = namedtuple("Register", ["addr", "offset", "mask"])

# Internal constants:
# Constants for addresses and masks to reduce RAM usage.
_0X00 = const(0x00)
_0X01 = const(0x01)
_0X02 = const(0x02)
_0X03 = const(0x03)
_0X04 = const(0x04)
_0X05 = const(0x05)
_0X06 = const(0x06)
_0X07 = const(0x07)
_0X08 = const(0x08)
_0X09 = const(0x09)
_0X0A = const(0x0A)
_0X0B = const(0x0B)
_0X0E = const(0x0E)
_0X0F = const(0x0F)
_0X10 = const(0x10)
_0X12 = const(0x12)
_0X1F = const(0x1F)
_0X20 = const(0x20)
_0X30 = const(0x30)
_0X3A = const(0x3A)
_0X3B = const(0x3B)
_0X3C = const(0x3C)
_0X3D = const(0x3D)
_0X3E = const(0x3E)
_0X3F = const(0x3F)
_0X40 = const(0x40)
_0X70 = const(0x70)
_0X80 = const(0x80)
_0XC0 = const(0xC0)
_0XE0 = const(0xE0)
_0XFF = const(0xFF)

_LIGHTNING_COUNT = (_0X01, _0X05, _0X09, _0X10)
_FREQ_DIVISOR = (_0X10, _0X20, _0X40, _0X80)


def _reg_value_from_choices(value, choices):
    """Return the index of a value from an iterable."""
    try:
        return choices.index(value)
    except ValueError as error:
        raise ValueError(
            "Select a value from %s" % ", ".join([str(x) for x in choices])
        ) from error


def _value_is_in_range(value, *, lo_limit, hi_limit):
    """Return the value if it is within the range lo_limit to hi_limit inclusive."""
    try:
        assert lo_limit <= value <= hi_limit
    except AssertionError as error:
        raise ValueError(
            "Value must be in the range %s to %s inclusive." % (lo_limit, hi_limit)
        ) from error
    return value


class AS3935:
    """Supports the Franklin AS3935 lightning detector chip via the SPI interface. Allows
    monitoring for lightning events by polling an 'interrupt' pin that is held high for one
    second after an event. Allows reading strength of the last strike and estimated
    distance to the storm front. Antenna trimming, clock calibration, etc. are also
    implemented.

    :param busio.SPI spi: The SPI bus connected to the chip.  Ensure SCK, MOSI, and MISO are
        connected.
    :param ~board.Pin cs: The pin connected to the chip's CS/chip select line.
    :param ~board.Pin interrupt_pin: The pin connected to the chip's interrupt line. Note
        that CircuitPython currently does not support interrupts, but the line is held high
        for at least one second per event, so it may be polled. Some single board computers,
        e.g. the Raspberry Pi, do support interrupts.
    :param int baudrate: Defaults to 2,000,000 which is the maximum supported by the chip. If
        another baudrate is selected, avoid +/- 500,000 as this will interfere with the chip's
        antenna.
    """

    # Global bufferS for SPI commands and address.
    _ADDR_BUFFER = bytearray(1)
    _DATA_BUFFER = bytearray(1)

    # Constants to make register values human readable in the code
    DATA_PURGE = _0X00  # 0x00 - Distance recalculated after purging old data.
    NOISE = _0X01  # 0x01 - INT_NH Noise level too high. Stays high while noise remains.
    DISTURBER = _0X04  # 0x04 - INT_D  Disturber detected.
    LIGHTNING = _0X08 # 0x08 - INT_L  Lightning strike.
    DIRECT_COMMAND = const(0x96)

    # AS3935 registers
    # _register_name = _Register(address, offset, mask)
    _pwd = _Register(_0X00, _0X00, _0X01)
    _afe_gb = _Register(_0X00, _0X01, _0X3E)
    _wdth = _Register(_0X01, _0X00, _0X0F)
    _nf_lev = _Register(_0X01, _0X04, _0X70)
    _srej = _Register(_0X02, _0X00, _0X0F)
    _min_num_ligh = _Register(_0X02, _0X04, _0X30)
    _cl_stat = _Register(_0X02, _0X06, _0X40)
    _int = _Register(_0X03, _0X00, _0X0F)
    _mask_dist = _Register(_0X03, _0X05, _0X20)
    _lco_fdiv = _Register(_0X03, _0X06, _0XC0)
    _s_lig_l = _Register(_0X04, _0X00, _0XFF)
    _s_lig_m = _Register(_0X05, _0X00, _0XFF)
    _s_lig_mm = _Register(_0X06, _0X00, _0X1F)
    _distance = _Register(_0X07, _0X00, _0X3F)
    _tun_cap = _Register(_0X08, _0X00, _0X0F)
    _disp_flags = _Register(_0X08, _0X05, _0XE0)
    _trco_calib = _Register(_0X3A, _0X06, _0XC0)
    _srco_calib = _Register(_0X3B, _0X06, _0XC0)
    _preset_default = _Register(_0X3C, _0X00, _0XFF)
    _calib_rco = _Register(_0X3D, _0X00, _0XFF)

    def __init__(self, spi, cs, *, interrupt_pin, baudrate=2_000_000):
        self._device = spi_dev.SPIDevice(
            spi, digitalio.DigitalInOut(cs), baudrate=baudrate, polarity=1, phase=0
        )
        self._as3935_startup_checks()
        self._interrupt_pin = digitalio.DigitalInOut(interrupt_pin)
        self._interrupt_pin.direction = digitalio.Direction.INPUT

    def _read_byte_in(self, register):
        """Read one byte from the selected address."""
        self._ADDR_BUFFER[0] = (
            register.addr & _0X3F
        ) | _0X40  # Set bits 15 and 14 to 01 - read
        with self._device as device:
            device.write(self._ADDR_BUFFER, end=1)
            device.readinto(self._DATA_BUFFER, end=1)
            return self._DATA_BUFFER[0]

    def _write_byte_out(self, register, data):
        """Write one byte to the selected register."""
        self._ADDR_BUFFER[0] = register.addr & _0X3F  # Set bits 15 and 14 to 00 - write
        self._DATA_BUFFER[0] = data
        with self._device as device:
            device.write(self._ADDR_BUFFER, end=1)
            device.write(self._DATA_BUFFER, end=1)

    def _get_register(self, register):
        """Read the current register byte, mask and shift the value."""
        return (self._read_byte_in(register) & register.mask) >> register.offset

    def _set_register(self, register, value):
        """Read the byte containing the register, mask in the new value and write out the byte."""
        byte = self._read_byte_in(register)
        byte &= ~register.mask
        byte |= (value << register.offset) & _0XFF
        self._write_byte_out(register, byte)

    @property
    def indoor(self):
        """bool: Get or set Indoor mode. This must be set to True if the sensor is used indoors.
        and False if the sensor is used outdoors.  Default is True.
        """
        # Register _afe_gb is set to 0x12 for Indoor mode and 0x0e for outdoor mode
        if self._get_register(self._afe_gb) == _0X12:
            return True
        return False

    @indoor.setter
    def indoor(self, value):
        assert isinstance(value, bool)
        if value:
            self._set_register(self._afe_gb, _0X12)
        else:
            self._set_register(self._afe_gb, _0X0E)

    @property
    def watchdog(self):
        """int: Watchdog threshold. Higher thresholds decrease triggers by disturbers
        but reduce sensitivity to lightning strikes.

        Integer in the range 0 - 10. Default is 2.
        """
        return self._get_register(self._wdth)

    @watchdog.setter
    def watchdog(self, value):
        self._set_register(
            self._wdth, _value_is_in_range(value, lo_limit=_0X00, hi_limit=_0X0A)
        )

    @property
    def noise_floor_limit(self):
        """int: Get or set the noise floor limit threshold. When this threshold is
        exceeded, an interrupt is issued. Higher values allow operation with
        higher background noise but decrease sensitivity.

        Integer in the range 0 - 7. Default is 2.
        """
        return self._get_register(self._nf_lev)

    @noise_floor_limit.setter
    def noise_floor_limit(self, value):
        self._set_register(
            self._nf_lev, _value_is_in_range(value, lo_limit=_0X00, hi_limit=_0X07)
        )

    @property
    def spike_threshold(self):
        """int: Get or set the spike rejection threshold. Higher values reduce
        false triggers but reduce sensitivity.

        Integer in the range 0 - 10. Default is 2.
        """
        return self._get_register(self._srej)

    @spike_threshold.setter
    def spike_threshold(self, value):
        assert _0X00 <= value <= _0X0B
        self._set_register(
            self._srej, _value_is_in_range(value, lo_limit=_0X00, hi_limit=_0X0B)
        )

    @property
    def energy(self):
        """int: The calculated energy of the last lightning strike. This is a
        dimensionless number.
        """
        mmsb = self._get_register(self._s_lig_mm)
        msb = self._get_register(self._s_lig_m)
        lsb = self._get_register(self._s_lig_l)
        return ((mmsb << 16) | (msb << 8) | lsb) & 0x3FFFFF

    @property
    def distance(self):
        """int: Estimated distance to the storm front (km).

        Returns None if storm front is out of range (> 40 km).
        """
        distance = self._get_register(self._distance)
        if distance == 0x3F:  # Storm out of range
            distance = None
        elif distance == 0x01:  # Storm overhead
            distance = 0
        return distance  # Distance in km

    @property
    def interrupt_status(self):
        """int: Status of the interrupt register.

        The following class constants are defined as helpers:
        DATA_PURGE = 0x00  # Distance recalculated after purging old data.
        NOISE = 0x01  # INT_NH Noise level too high. Stays high while noise remains.
        DISTURBER = 0x04  # INT_D  Disturber detected.
        LIGHTNING = 0x08  # INT_L  Lightning strike.

        This register is cleared after it is read.
        """
        # Wait a minimum of 2 ms between the interrupt pin going high and reading the register.
        time.sleep(0.0002)
        return self._get_register(self._int)

    @property
    def disturber_mask(self):
        """bool: Disturber mask. If the mask is True, disturber events do not
        cause interrupts. Default is False.
        """
        return bool(self._get_register(self._mask_dist))

    @disturber_mask.setter
    def disturber_mask(self, value):
        # Set the register value to 0x01 to suppress disturber event interrupts.
        # Set the register value to 0x00 to allow disturber event interrupts.
        assert isinstance(value, bool)
        self._set_register(self._mask_dist, int(value))

    @property
    def strike_count_threshold(self):
        """int: Lightning strike count threshold. The minimum number of
        lightning events before the interrupt is triggered. This threshold is
        reset to the default value of 1 after being triggered.

        Threshold may be 1, 5, 9, or 16. Default is 1.
        """
        # Convert the register value to the threshold.
        return _LIGHTNING_COUNT[self._get_register(self._min_num_ligh)]

    @strike_count_threshold.setter
    def strike_count_threshold(self, value):
        self._set_register(
            self._min_num_ligh, _reg_value_from_choices(value, _LIGHTNING_COUNT)
        )

    def clear_stats(self):
        """Clear statistics from lightning distance emulation block. This resets the
        data used to calculate the distance to the storm fron.
        """
        self._set_register(self._cl_stat, 0x01)
        self._set_register(self._cl_stat, 0x00)
        self._set_register(self._cl_stat, 0x01)

    @property
    def power_down(self):
        """bool: Power status. If True, the unit is powered off although the SPI bus remains
        active."""
        return bool(self._get_register(self._pwd))

    @power_down.setter
    def power_down(self, value):
        assert isinstance(value, bool)
        if value:
            self._set_register(self._pwd, 0x01)
        elif self.power_down:
            # Only do this if the power_down mode is already set as clocks get calibrated.
            self._set_register(self._pwd, 0x00)
            # RCO clocks need to be calibrated when powering back up from a power_down = True
            # Procedure as per AS3935 datasheet
            self._calibrate_clocks()
            self._set_register(self._disp_flags, 0x02)
            time.sleep(0.002)
            self._set_register(self._disp_flags, 0x00)

    @property
    def freq_divisor(self):
        """int: Antenna frequency divisor. The antenna resonant frequency is divided by
        this value whenever it is output as a square wave on the interrupt pin.

        Value must be one of 16, 32, 64, or 128. Default is 16.
        """
        # Convert the register value to the divisor.
        return _FREQ_DIVISOR[self._get_register(self._lco_fdiv)]

    @freq_divisor.setter
    def freq_divisor(self, value):
        self._set_register(
            self._lco_fdiv, _reg_value_from_choices(value, _FREQ_DIVISOR)
        )

    @property
    def output_antenna_freq(self):
        """bool: When True, the antenna resonant frequency is divided by the freq_divisor
        and output as a square wave on the interrupt pin. Default is False.
        """
        # Convert bit 3 status to bool
        return self._get_register(self._disp_flags) == 4

    @output_antenna_freq.setter
    def output_antenna_freq(self, value):
        assert isinstance(value, bool)
        # Set the register value to 0x04 to enable antenna tuning mode.
        # Set the register value to 0x00 to disable antenna tuning mode.
        self._set_register(self._disp_flags, int(value) << 2)

    @property
    def output_srco(self):
        """bool: When True, output the SRCO clock signal on the interrupt pin.
        Default is False.
        """
        # Convert bit 2 status to bool
        return self._get_register(self._disp_flags) == 2

    @output_srco.setter
    def output_srco(self, value):
        assert isinstance(value, bool)
        # Set the register value to 0x02 to output SRCO clock to the interrupt pin.
        # Set the register value to 0x00 to allow normal interrupt operation.
        self._set_register(self._disp_flags, int(value) << 1)

    @property
    def output_trco(self):
        """bool: When True, output the TRCO clock signal on the interrupt pin.
        Default is False.
        """
        # Convert bit 1 status to bool
        return self._get_register(self._disp_flags) == 1

    @output_trco.setter
    def output_trco(self, value):
        assert isinstance(value, bool)
        # Set the register value to 0x01 to output SRCO clock to the interrupt pin.
        # Set the register value to 0x00 to allow normal interrupt operation.
        self._set_register(self._disp_flags, int(value))

    @property
    def tuning_capacitance(self):
        """int: The tuning capacitance for the RLC antenna in pF. This capacitance
        is added to the antenna to tune it within 3.5 % of 500 kHz (483 - 517 kHz).

        Values must be in the range 0 - 120. Any of these values may be set, but the
        capacitance is in steps of 8, so values less than 120 will be rounded down to
        the nearest step. Default is 0.
        """
        return self._get_register(self._tun_cap) * 8

    @tuning_capacitance.setter
    def tuning_capacitance(self, value):
        self._set_register(
            self._tun_cap, _value_is_in_range(value, lo_limit=_0X00, hi_limit=120) // 8
        )

    def _check_clock_calibration(self):
        """Check that clock calibration was successful."""
        start = time.monotonic()
        trco_result, srco_result = _0X00, _0X00
        while not (trco_result and srco_result):
            if time.monotonic() - start > _0X01:
                raise OSError("Unable to communicate with the sensor. Check your wiring.")
            trco_result = self._get_register(self._trco_calib)
            srco_result = self._get_register(self._srco_calib)
        if trco_result == _0X01 or srco_result == srco_result == _0X01:
            raise RuntimeError("AS3935 RCO clock calibration failed.")


    def _calibrate_clocks(self):
        """Recalibrate the internal clocks."""
        # Send the direct command to the CALIB_RCO register to start automatic RCO calibration
        self._set_register(self._calib_rco, self.DIRECT_COMMAND)
        self._check_clock_calibration()

    def reset(self):
        """Reset all the settings to the manufacturer's defaults."""
        # Send the direct command to the PRESET_DEFAUTLT register to start reset settings
        self._set_register(self._preset_default, self.DIRECT_COMMAND)

    @property
    def interrupt_set(self):
        """bool: The state of the interrupt pin. Returns True if the pin is high,
        False if the pin is low and None if the pin is set to output a clock or
        antenna frequency.

        The interrupt pin is held high for 1.0 second after a lightning event.
        The interrupt pin is held high for 1.5 seconds after a disturber event.
        The interrupt pin is held high for the duration of high noise.
        """
        if self._get_register(self._disp_flags):
            return None
        return self._interrupt_pin.value

    def _as3935_startup_checks(self):
        """Check communication with the AS3935 and confirm clocks are calibrated."""
        # With no sensor connected, reading the SPI Device returns 0x00, so read the
        # indoor register that should never return 0x00 and raise an exception if
        # required
        self.reset()  # Put the sensor into a known, default state.
        self._check_clock_calibration()
