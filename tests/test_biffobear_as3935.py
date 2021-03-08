# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT

# Many Pylnt conventions are broken for the sake of test readability
# Others fail because Pylint doesn't understand Pytest.
# Therefore skip this file.
# pylint: skip-file

import time
import inspect
from collections import namedtuple
from random import random
from unittest.mock import PropertyMock
import pytest
from CircuitPython_AS3935 import biffobear_as3935 as as3935


@pytest.fixture
def get_reg(mocker):
    return mocker.patch.object(as3935.AS3935_Sensor, "_get_register", autospec=True)


@pytest.fixture
def set_reg(mocker):
    return mocker.patch.object(as3935.AS3935_Sensor, "_set_register", autospec=True)


@pytest.fixture
def test_device(mocker):
    # Returns an instance of the AS3935 driver with SDIDevice patched.
    mocker.patch.object(as3935.digitalio, "DigitalInOut")
    mocker.patch.object(as3935.AS3935_Sensor, "_startup_checks", return_value=None)
    return as3935.AS3935_Sensor(interrupt_pin="int_pin")


@pytest.fixture
def test_register():
    # Returns an instance of the Register named tuple
    return as3935._Register(0x01, 0x04, 0b0111_0000)


def test_register_onstants():
    assert as3935.AS3935_Sensor._0X00 == 0x00
    assert as3935.AS3935_Sensor._0X01 == 0x01
    assert as3935.AS3935_Sensor._0X02 == 0x02
    assert as3935.AS3935_Sensor._0X03 == 0x03
    assert as3935.AS3935_Sensor._0X04 == 0x04
    assert as3935.AS3935_Sensor._0X05 == 0x05
    assert as3935.AS3935_Sensor._0X06 == 0x06
    assert as3935.AS3935_Sensor._0X07 == 0x07
    assert as3935.AS3935_Sensor._0X08 == 0x08
    assert as3935.AS3935_Sensor._0X0B == 0x0B
    assert as3935.AS3935_Sensor._0X0F == 0x0F
    assert as3935.AS3935_Sensor._0X10 == 0x10
    assert as3935.AS3935_Sensor._0X12 == 0x12
    assert as3935.AS3935_Sensor._0X20 == 0x20
    assert as3935.AS3935_Sensor._0X3F == 0x3F
    assert as3935.AS3935_Sensor._0X40 == 0x40
    assert as3935.AS3935_Sensor._0XC0 == 0xC0
    assert as3935.AS3935_Sensor._0XFF == 0xFF


def test_other_constants():
    assert as3935.AS3935_Sensor._LIGHTNING_COUNT == (1, 5, 9, 16)
    assert as3935.AS3935_Sensor._FREQ_DIVISOR == (16, 32, 64, 128)
    # 0x00 - Distance recalculated after purging old data.
    assert as3935.AS3935_Sensor.DATA_PURGE == 0x00
    # 0x01 - INT_NH Noise level too high. Stays high while noise remains.
    assert as3935.AS3935_Sensor.NOISE == 0x01
    # 0x04 - INT_D  Disturber detected.
    assert as3935.AS3935_Sensor.DISTURBER == 0x04
    # 0x08 - INT_L  Lightning strike.
    assert as3935.AS3935_Sensor.LIGHTNING == 0x08
    # 0x96 is sent to initiate a reset or a clock calibration.
    assert as3935.AS3935_Sensor.DIRECT_COMMAND == 0x96


def test_address_and_data_command_buffers():
    assert isinstance(as3935._BUFFER, bytearray)
    assert len(as3935._BUFFER) == 2


@pytest.mark.parametrize(
    "register, addr, offset, mask",
    [
        (as3935.AS3935_Sensor._PWD, 0, 0, 0b0000_0001),
        (as3935.AS3935_Sensor._AFE_GB, 0, 1, 0b0011_1110),
        (as3935.AS3935_Sensor._WDTH, 1, 0, 0b0000_1111),
        (as3935.AS3935_Sensor._NF_LEV, 1, 4, 0b0111_0000),
        (as3935.AS3935_Sensor._SREJ, 2, 0, 0b0000_1111),
        (as3935.AS3935_Sensor._MIN_NUM_LIGH, 2, 4, 0b0011_0000),
        (as3935.AS3935_Sensor._CL_STAT, 2, 6, 0b0100_0000),
        (as3935.AS3935_Sensor._INT, 3, 0, 0b0000_1111),
        (as3935.AS3935_Sensor._MASK_DIST, 3, 5, 0b0010_0000),
        (as3935.AS3935_Sensor._LCO_FDIV, 3, 6, 0b1100_0000),
        (as3935.AS3935_Sensor._S_LIG_L, 4, 0, 0b1111_1111),
        (as3935.AS3935_Sensor._S_LIG_M, 5, 0, 0b1111_1111),
        (as3935.AS3935_Sensor._S_LIG_MM, 6, 0, 0b0001_1111),
        (as3935.AS3935_Sensor._DISTANCE, 7, 0, 0b0011_1111),
        (as3935.AS3935_Sensor._TUN_CAP, 8, 0, 0b0000_1111),
        (as3935.AS3935_Sensor._DISP_FLAGS, 8, 5, 0b1110_0000),
        (as3935.AS3935_Sensor._TRCO_CALIB, 58, 6, 0b1100_0000),
        (as3935.AS3935_Sensor._SRCO_CALIB, 59, 6, 0b1100_0000),
        (as3935.AS3935_Sensor._PRESET_DEFAULT, 60, 0, 0b1111_1111),
        (as3935.AS3935_Sensor._CALIB_RCO, 61, 0, 0b1111_1111),
    ],
)
def test_register_settings(register, addr, offset, mask):
    # Compare register tuples
    assert register.addr == addr
    assert register.offset == offset
    assert register.mask == mask


def test_init_method_called_with_correct_args(mocker):
    mock_init = mocker.patch.object(
        as3935.AS3935_Sensor, "__init__", autospec=True, return_value=None
    )
    test_as3935 = as3935.AS3935_Sensor(interrupt_pin="pin")
    mock_init.assert_called_once_with(test_as3935, interrupt_pin="pin")


@pytest.mark.parametrize(
    "int_pin, int_pin_out",
    [("int_pin1", "int_pin_out1"), ("int_pin2", "int_pin_out2")],
)
def test_init_calls(mocker, int_pin, int_pin_out):
    mock_int_pin = mocker.Mock(name=int_pin)
    mock_digitalinout = mocker.patch.object(
        as3935.digitalio, "DigitalInOut", return_value=mock_int_pin
    )
    mock_startup_checks = mocker.patch.object(
        as3935.AS3935_Sensor, "_startup_checks", autospec=True
    )
    test_as3935 = as3935.AS3935_Sensor(interrupt_pin=mock_int_pin)
    mock_digitalinout.assert_called_once_with(mock_int_pin)
    # Confirm DigitalInOut object from interrupt_pin arg is assigned to self.interrupt_pin
    assert test_as3935._interrupt_pin == mock_int_pin
    # Check that DigitalInOUt called with interrupt_pin from args
    mock_digitalinout.assert_called_once_with(mock_int_pin)
    # Check that interrupt pin direction is correctly set
    # No test yet
    # Check that startup checks are run
    mock_startup_checks.assert_called_once()


def test_read_byte_in_has_same_signature_as_subclasses():
    assert inspect.signature(as3935.AS3935_Sensor._read_byte_in) == inspect.signature(
        as3935.AS3935_I2C._read_byte_in
    )
    assert inspect.signature(as3935.AS3935_Sensor._read_byte_in) == inspect.signature(
        as3935.AS3935._read_byte_in
    )


def test_write_byte_out_has_same_signature_as_subclasses():
    assert inspect.signature(as3935.AS3935_Sensor._write_byte_out) == inspect.signature(
        as3935.AS3935_I2C._write_byte_out
    )
    assert inspect.signature(as3935.AS3935_Sensor._write_byte_out) == inspect.signature(
        as3935.AS3935._write_byte_out
    )


@pytest.mark.parametrize(
    "register_byte, return_byte", [(0xFF, 0x07), (0x00, 0x00), (0x55, 0x05)]
)
def test_get_register(mocker, test_device, test_register, register_byte, return_byte):
    mock_read_byte_in = mocker.patch.object(
        as3935.AS3935_Sensor, "_read_byte_in", autospec=True, return_value=register_byte
    )
    result = test_device._get_register(test_register)
    mock_read_byte_in.assert_called_once_with(test_device, test_register)
    assert result == return_byte


@pytest.mark.parametrize(
    "byte_in, register_value, byte_out",
    [(0xFF, 0x00, 0x8F), (0x00, 0x07, 0x70), (0x55, 0x05, 0x55)],
)
def test_set_register(
    mocker, test_device, test_register, byte_in, register_value, byte_out
):
    mock_read_byte_in = mocker.patch.object(
        as3935.AS3935_Sensor, "_read_byte_in", autospec=True, return_value=byte_in
    )
    mock_write_byte_out = mocker.patch.object(
        as3935.AS3935_Sensor, "_write_byte_out", autospec=True
    )
    test_device._set_register(test_register, register_value)
    mock_read_byte_in.assert_called_once_with(test_device, test_register)
    mock_write_byte_out.assert_called_once_with(test_device, test_register, byte_out)


@pytest.mark.parametrize("value, register_value", [(2, 0), (3, 1), (5, 2)])
def test_reg_value_from_choices_returns_correct_value(value, register_value):
    assert as3935._reg_value_from_choices(value, (2, 3, 5)) == register_value


@pytest.mark.parametrize("bad_arg", [1, "x", 1.1])
def test_reg_value_from_choices_handles_invalid_args(bad_arg):
    with pytest.raises(ValueError):
        as3935._reg_value_from_choices(bad_arg, (2, 3, 5))


@pytest.mark.parametrize("value", [0, 5, 8])
def test_value_is_in_range_returns_correct_value(value):
    assert as3935._value_is_in_range(value, lo_limit=0, hi_limit=8) == value


@pytest.mark.parametrize("value", [1.1, "9", b"a"])
def test_value_is_in_range_handles_incorrect_type(value):
    with pytest.raises(TypeError):
        as3935._value_is_in_range(value, lo_limit=0, hi_limit=8)


@pytest.mark.parametrize("value", [-1, 9])
def test_value_is_in_range_handles_incorrect_value(value):
    with pytest.raises(ValueError):
        as3935._value_is_in_range(value, lo_limit=0, hi_limit=8)


@pytest.mark.parametrize("register_value, result", [(0x12, True), (0x0E, False)])
def test_indoor_getter(get_reg, test_device, register_value, result):
    # The register value is 0x12 if Indoor mode is set
    get_reg.return_value = register_value
    assert test_device.indoor == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._AFE_GB)


@pytest.mark.parametrize("value, register_value", [(True, 0x12), (False, 0x0E)])
def test_indoor_setter(set_reg, test_device, value, register_value):
    # Set the register value to 0x12 for Indoor mode and 0x0e for outdoor mode
    test_device.indoor = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._AFE_GB, register_value
    )
    # Test that none boolean values are rejected
    with pytest.raises(AssertionError):
        test_device.indoor = "1"


@pytest.mark.parametrize("value", [0x00, 0x04])
def test_watchdog_getter(get_reg, test_device, value):
    get_reg.return_value = value
    assert test_device.watchdog == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._WDTH)


@pytest.mark.parametrize("value, out_of_range_value", [(0x00, -1), (0x0A, 0x0B)])
def test_watchdog_setter(set_reg, test_device, value, out_of_range_value):
    # Test with maximum and minimum allowed values
    test_device.watchdog = value
    set_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._WDTH, value)
    # Test with out_of_range_values just outside acceptable range
    with pytest.raises(ValueError):
        test_device.watchdog = out_of_range_value


@pytest.mark.parametrize("value", [0x00, 0x04])
def test_noise_floor_limit_getter(get_reg, test_device, value):
    get_reg.return_value = value
    assert test_device.noise_floor_limit == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._NF_LEV)


@pytest.mark.parametrize("value, out_of_range_value", [(0x00, -1), (0x07, 0x08)])
def test_noise_floor_limit_setter(set_reg, test_device, value, out_of_range_value):
    # Test with maximum and minimum allowed values
    test_device.noise_floor_limit = value
    set_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._NF_LEV, value)
    # Test with out_of_range_values just outside acceptable range
    with pytest.raises(ValueError):
        test_device.noise_floor_limit = out_of_range_value


@pytest.mark.parametrize("value", [0x00, 0x04])
def test_spike_threshold_getter(get_reg, test_device, value):
    get_reg.return_value = value
    assert test_device.spike_threshold == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._SREJ)


@pytest.mark.parametrize("value, out_of_range_value", [(0x00, -1), (0x0B, 0x0C)])
def test_spike_threshold_setter(set_reg, test_device, value, out_of_range_value):
    # Test with maximum and minimum allowed values
    test_device.spike_threshold = value
    set_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._SREJ, value)
    # Test with out_of_range_values just outside acceptable range
    with pytest.raises(ValueError):
        test_device.spike_threshold = out_of_range_value


def test_enery_getter(mocker, get_reg, test_device):
    get_reg.side_effect = [0x06, 0x55, 0x44]
    expected_calls = [
        mocker.call(test_device, as3935.AS3935_Sensor._S_LIG_MM),
        mocker.call(test_device, as3935.AS3935_Sensor._S_LIG_M),
        mocker.call(test_device, as3935.AS3935_Sensor._S_LIG_L),
    ]
    assert test_device.energy == 0x065544
    assert get_reg.call_count == 3
    assert get_reg.call_args_list == expected_calls


@pytest.mark.parametrize("register_value, value", [(0x01, 0), (0x11, 17), (0x3F, None)])
def test_distance_getter(get_reg, test_device, register_value, value):
    # Register = 0x01, storm overhead, register = 0x3f out of range, other values in km
    get_reg.return_value = register_value
    assert test_device.distance == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._DISTANCE)


@pytest.mark.parametrize("value", [0x00, 0x01, 0x04, 0x08])
def test_interrupt_status_getter(get_reg, test_device, value):
    get_reg.return_value = value
    assert test_device.interrupt_status == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._INT)


@pytest.mark.parametrize("register_value, result", [(0x01, True), (0x00, False)])
def test_disturber_mask_getter(get_reg, test_device, register_value, result):
    # If the register is set disturber events are ignored. If clear, disturbers cause interrupts.
    get_reg.return_value = register_value
    assert test_device.disturber_mask == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._MASK_DIST)


@pytest.mark.parametrize("value, register_value", [(True, 0x01), (False, 0x00)])
def test_disturber_mask_setter(set_reg, test_device, value, register_value):
    # Set the register value to 0x01 to suppress disturber event interrupts.
    # Set the register value to 0x00 to allow disturber event interrupts.
    test_device.disturber_mask = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._MASK_DIST, register_value
    )
    # Test that none bool values are rejected
    with pytest.raises(AssertionError):
        test_device.disturber_mask = "1"


@pytest.mark.parametrize(
    "register_value, value", [(0x00, 1), (0x01, 5), (0x02, 9), (0x03, 16)]
)
def test_strike_count_threshold_getter(get_reg, test_device, register_value, value):
    get_reg.return_value = register_value
    assert test_device.strike_count_threshold == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._MIN_NUM_LIGH)


@pytest.mark.parametrize(
    "value, out_of_range_value, register_value",
    [(1, -1, 0x00), (5, 99, 0x01), (9, 8, 0x02), (16, 2, 0x03)],
)
def test_strike_count_threshold_setter(
    set_reg, test_device, value, out_of_range_value, register_value
):
    # Test with all allowable threshold values (1, 5, 9, 16)
    test_device.strike_count_threshold = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._MIN_NUM_LIGH, register_value
    )
    # Test with out_of_range_values
    with pytest.raises(ValueError):
        test_device.strike_count_threshold = out_of_range_value


def test_clear_stats(mocker, set_reg, test_device):
    expected_calls = [
        mocker.call(test_device, as3935.AS3935_Sensor._CL_STAT, 0x01),
        mocker.call(test_device, as3935.AS3935_Sensor._CL_STAT, 0x00),
        mocker.call(test_device, as3935.AS3935_Sensor._CL_STAT, 0x01),
    ]
    test_device.clear_stats()
    assert set_reg.call_count == 3
    assert set_reg.call_args_list == expected_calls


@pytest.mark.parametrize("register_value, result", [(0x01, True), (0x00, False)])
def test_power_down_getter(get_reg, test_device, register_value, result):
    # If the register is set the unit is powered off.
    get_reg.return_value = register_value
    assert test_device.power_down == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._PWD)


def test_power_down_setter_true(set_reg, test_device):
    # Test power_down True and False seperately because False is complicated.
    test_device.power_down = True
    set_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._PWD, 0x01)


def test_power_down_setter_false_and_power_is_down(
    mocker, set_reg, get_reg, test_device
):
    # If turning on power after power_down was previously set, clocks must be calibrated.
    get_reg.return_value = 0x01
    mock_calibrate_clocks = mocker.patch.object(
        as3935.AS3935_Sensor, "calibrate_clocks", autospec=True
    )
    mock_check_clock_calibration = mocker.patch.object(
        as3935.AS3935_Sensor, "_check_clock_calibration", autospec=True
    )
    mock_sleep = mocker.patch.object(as3935.time, "sleep")
    expected_calls = [
        mocker.call(test_device, as3935.AS3935_Sensor._PWD, 0x00),  # Power down
        mocker.call(
            test_device, as3935.AS3935_Sensor._DISP_FLAGS, 0x02
        ),  # Set DISP_SRCO
        mocker.call(
            test_device, as3935.AS3935_Sensor._DISP_FLAGS, 0x00
        ),  # Clear DISP_SRCO
    ]
    test_device.power_down = False
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._PWD)
    assert set_reg.call_args_list == expected_calls
    mock_calibrate_clocks.assert_called_once()
    mock_check_clock_calibration.assert_called_once()
    mock_sleep.assert_called_once_with(0.002)


def test_power_down_setter_false_and_power_is_up(mocker, set_reg, get_reg, test_device):
    # If turning on power with power already on then do nothing.
    get_reg.return_value = 0x00
    mock_calibrate_clocks = mocker.patch.object(
        as3935.AS3935_Sensor, "calibrate_clocks", autospec=True
    )
    test_device.power_down = False
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._PWD)
    set_reg.assert_not_called()
    mock_calibrate_clocks.assert_not_called()


def test_power_down_setter_raises_an_error_when_called_with_invalid_args(test_device):
    with pytest.raises(AssertionError):
        test_device.power_down = "1"


@pytest.mark.parametrize(
    "register_value, value", [(0x00, 16), (0x01, 32), (0x02, 64), (0x03, 128)]
)
def test_freq_divisor_getter(get_reg, test_device, register_value, value):
    get_reg.return_value = register_value
    assert test_device.freq_divisor == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._LCO_FDIV)


@pytest.mark.parametrize(
    "value, out_of_range_value, register_value",
    [(16, -1, 0x00), (32, 99, 0x01), (64, 8, 0x02), (128, 222, 0x03)],
)
def test_freq_divisor_setter(
    set_reg, test_device, value, out_of_range_value, register_value
):
    # Test with all allowable threshold values (1, 5, 9, 16)
    test_device.freq_divisor = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._LCO_FDIV, register_value
    )
    # Test with out_of_range_values
    with pytest.raises(ValueError):
        test_device.freq_divisor = out_of_range_value


@pytest.mark.parametrize("register_value, result", [(0x04, True), (0x00, False)])
def test_output_antenna_freq_getter(get_reg, test_device, register_value, result):
    # Set the register value to 0x01 to enable antenna tuning mode.
    # Set the register value to 0x00 to disable antenna tuning mode.
    get_reg.return_value = register_value
    assert test_device.output_antenna_freq == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._DISP_FLAGS)


@pytest.mark.parametrize("value, register_value", [(True, 0x04), (False, 0x00)])
def test_output_antenna_freq_setter(set_reg, test_device, value, register_value):
    # Set the register value to 0x01 to enable antenna tuning mode.
    # Set the register value to 0x00 to disable antenna tuning mode.
    test_device.output_antenna_freq = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._DISP_FLAGS, register_value
    )
    # Test that none bool values are rejected
    with pytest.raises(AssertionError):
        test_device.output_antenna_freq = "1"


@pytest.mark.parametrize("register_value, result", [(0x02, True), (0x00, False)])
def test_output_srco_getter(get_reg, test_device, register_value, result):
    # If the register is set the SRCO clock is output on the interrupt pin.
    get_reg.return_value = register_value
    assert test_device.output_srco == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._DISP_FLAGS)


@pytest.mark.parametrize("value, register_value", [(True, 0x02), (False, 0x00)])
def test_output_srco_setter(set_reg, test_device, value, register_value):
    # Set the register value to 0x01 to output SRCO clock on interrupt pin.
    # Set the register value to 0x00 to allow normal interrupt operation.
    test_device.output_srco = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._DISP_FLAGS, register_value
    )
    # Test that none bool values are rejected
    with pytest.raises(AssertionError):
        test_device.disturber_mask = "1"


@pytest.mark.parametrize("register_value, result", [(0x01, True), (0x00, False)])
def test_output_trco_getter(get_reg, test_device, register_value, result):
    # If the register is set the TRCO clock is output on the interrupt pin.
    get_reg.return_value = register_value
    assert test_device.output_trco == result
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._DISP_FLAGS)


@pytest.mark.parametrize("value, register_value", [(True, 0x01), (False, 0x00)])
def test_output_trco_setter(set_reg, test_device, value, register_value):
    # Set the register value to 0x01 to output TRCO clock on interrupt pin.
    # Set the register value to 0x00 to allow normal interrupt operation.
    test_device.output_trco = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._DISP_FLAGS, register_value
    )
    # Test that none bool values are rejected
    with pytest.raises(AssertionError):
        test_device.disturber_mask = "1"


@pytest.mark.parametrize(
    "register_value, value", [(0x00, 0), (0x01, 8), (0x0A, 80), (0x0F, 120)]
)
def test_tuning_capacitance_getter(get_reg, test_device, register_value, value):
    get_reg.return_value = register_value
    assert test_device.tuning_capacitance == value
    get_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._TUN_CAP)


@pytest.mark.parametrize(
    "value, out_of_range_value, register_value",
    [(0, -1, 0x00), (8, 121, 0x01), (9, 150, 0x01), (120, 222, 0x0F)],
)
def test_tuning_capacitance_setter(
    set_reg, test_device, value, out_of_range_value, register_value
):
    # Test tuning capacitor settings in range 0 - 120 // 8 i.e. 0x00 - 0x0f
    test_device.tuning_capacitance = value
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._TUN_CAP, register_value
    )
    # Test with out_of_range_values
    with pytest.raises(ValueError):
        test_device.tuning_capacitance = out_of_range_value


def test_calibrate_clocks_calls_correct_register_then_checks_calibration(
    mocker, set_reg, test_device
):
    mock_check_clock_calibration = mocker.patch.object(
        as3935.AS3935_Sensor, "_check_clock_calibration"
    )
    test_device.calibrate_clocks()
    set_reg.assert_called_once_with(test_device, as3935.AS3935_Sensor._CALIB_RCO, 0x96)
    mock_check_clock_calibration.assert_called_once()


def test_check_clock_calibration_waits_for_calibration_to_finish(
    mocker, set_reg, get_reg, test_device
):
    # Calibration complete when TRCO_CALIB_DONE and SRCO_CALIB_DONE are set
    get_reg.side_effect = [0x00, 0x00, 0x00, 0x02, 0x02, 0x02]
    expected_calls = [
        mocker.call(test_device, as3935.AS3935_Sensor._TRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._SRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._TRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._SRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._TRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._SRCO_CALIB),
    ]
    test_device._check_clock_calibration()
    assert get_reg.call_args_list == expected_calls


@pytest.mark.parametrize("side_effects", [[0x02, 0x01], [0x01, 0x02], [0x01, 0x01]])
def test_check_clock_calibration_raises_exception_when_a_calibration_fails(
    mocker, set_reg, get_reg, test_device, side_effects
):
    # TRCO_CALIB_NOK and SRCO_CALIB_NOK are set if the respective calibration failed
    get_reg.side_effect = side_effects
    expected_calls = [
        mocker.call(test_device, as3935.AS3935_Sensor._TRCO_CALIB),
        mocker.call(test_device, as3935.AS3935_Sensor._SRCO_CALIB),
    ]
    with pytest.raises(RuntimeError):
        test_device._check_clock_calibration()
    assert get_reg.call_args_list == expected_calls


def test_check_clock_calibration_raises_exception_for_timeout(
    mocker, set_reg, get_reg, test_device
):
    # This tests that a TimeoutError is raised if the calibration isn't complete after 1 second.
    get_reg.return_value = 0
    # Return times for start, 1 second after start and then a bit more.
    mock_monotonic = mocker.patch.object(
        as3935.time, "monotonic", side_effect=[1000, 1001, 1001.01]
    )
    with pytest.raises(OSError):
        test_device._check_clock_calibration()


def test_reset(test_device, set_reg):
    test_device.reset()
    set_reg.assert_called_once_with(
        test_device, as3935.AS3935_Sensor._PRESET_DEFAULT, 0x96
    )


@pytest.mark.parametrize(
    "pin_value, reg_value, return_value",
    [(True, 0x00, True), (False, 0x00, False), (True, 0x1, None), (False, 0x04, None)],
)
def test_interrupt_set(
    mocker, get_reg, test_device, pin_value, reg_value, return_value
):
    type(test_device._interrupt_pin).value = PropertyMock(return_value=pin_value)
    get_reg.return_value = reg_value
    assert test_device.interrupt_set is return_value


def test_startup_checks(mocker):
    mock_init = mocker.patch.object(as3935.AS3935_Sensor, "__init__", return_value=None)
    mock_reset = mocker.patch.object(
        as3935.AS3935_Sensor, "reset", autospec=True, return_value=None
    )
    mock_calibrate_clocks = mocker.patch.object(
        as3935.AS3935_Sensor, "calibrate_clocks", autospec=True, return_value=None
    )
    mock_check_clock_calibration = mocker.patch.object(
        as3935.AS3935_Sensor,
        "_check_clock_calibration",
        autospec=True,
        return_value=None,
    )
    # Confirm reset and check clock calibration functions were called
    test_device = as3935.AS3935_Sensor(bus="bus", interrupt_pin="pin")
    test_device._startup_checks()
    mock_reset.assert_called_once()
    mock_calibrate_clocks.assert_called_once()
    mock_check_clock_calibration.assert_called_once()
