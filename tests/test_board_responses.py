# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT
"""These tests are run with a sensor connected to confirm that the correct
responses are received from the sensor.

The try - except clauses and an if __name__ == "__main__" allow the code to be
run with pytest on a Raspberry Pi or as a stand alone file copied into main.py
on a CircuitPython board. To run on a board also copy 'biffobear_as3935.py' to
the lib folder.
"""

# Many Pylnt conventions are broken for the sake of test readability
# Others fail because Pylint doesn't understand Pytest.
# Therefore skip this file.
# pylint: skip-file

import os
import time
try:
    import pytest
except ImportError:
    pass
import board
try:
    from CircuitPython_AS3935 import biffobear_as3935 as as3935
except ImportError:
    import biffobear_as3935 as as3935

# All tests skipped unless environment variable SENSOR_ATTACHED is set to any value.
try:
    sensor_attached = int(os.environ["SENSOR_ATTACHED"])
except (KeyError, AttributeError):
    sensor_attached = False

try:
    pytestmark = pytest.mark.skipif(
        sensor_attached is False, reason="No as3935 board connected."
    )
except NameError:
    pass


device = None


def setup_module():
    # Returns an instance of the AS3935 driver
    global device
    spi = board.SPI()
    try:
        cs = board.D24
        interrupt = board.D25
    except AttributeError:
        cs = board.D5
        interrupt = board.D7

    device = as3935.AS3935_SPI(spi, cs, interrupt_pin=interrupt)
    device.reset()


def teardown_module():
    # Reset the chip between runs for consistent test results
    device.reset()


def test_indoor_outdoor():
    assert device.indoor is True  # Chip default
    device.indoor = False
    assert device.indoor is False


def test_power_down():
    assert device.power_down is False  # Chip default
    device.power_down = True
    assert device.power_down is True
    device.power_down = False
    assert device.power_down is False


def test_noise_floor_level():
    assert device.noise_floor_limit == 0x02  # Chip default
    # Test possible values
    for level in range(8):
        device.noise_floor_limit = level
        assert device.noise_floor_limit == level


def test_watchdog():
    assert device.watchdog == 0x02  # Chip default
    # Test possible values
    for level in range(11):
        device.watchdog = level
        assert device.watchdog == level


def test_spike_rejection():
    assert device.spike_threshold == 0x02  # Chip default
    # Test possible values
    for level in range(12):
        device.spike_threshold = level
        assert device.spike_threshold == level


def test_disturber_mask():
    assert device.disturber_mask is False  # Chip default
    device.disturber_mask = True
    assert device.disturber_mask is True


def test_strike_count_threshold():
    assert device.strike_count_threshold == 1
    # Test possible values
    for level in (1, 5, 9, 16):
        device.strike_count_threshold = level
        assert device.strike_count_threshold == level


def test_freq_divisor():
    assert device.freq_divisor == 16  # Chip default
    # Test possible values
    for divisor in (16, 32, 64, 128):
        device.freq_divisor = divisor
        assert device.freq_divisor == divisor


def test_output_antenna_freq():
    assert device.output_antenna_freq is False
    device.output_antenna_freq = True
    assert device.output_antenna_freq is True


def test_output_srco():
    assert device.output_srco is False  # Chip default
    device.output_srco = True
    assert device.output_srco is True


def test_output_trco():
    assert device.output_trco is False  # Chip default
    device.output_trco = True
    assert device.output_trco is True


def test_tuning_capacitance():
    assert device.tuning_capacitance == 0  # Chip default
    # Test possible values
    for capacitance in range(0, 128, 8):
        device.tuning_capacitance = capacitance
        assert device.tuning_capacitance == capacitance


def test_reset():
    # Set a none default value
    device.freq_divisor = 32
    assert device.freq_divisor == 32
    device.reset()
    # Confirm that is reset to default
    assert device.freq_divisor == 16  # Chip default


def test_commands_which_do_not_change_readable_values():
    # Call to see if an exception is raised
    device.clear_stats()
    device._calibrate_clocks()


def test_registers_with_unpredictable_states():
    # Just read them to see if an error occurs since value depends on presence of lightning.
    device.energy
    device.distance
    device.interrupt_status


def test_read_interrupt_pin():
    # The state of the pin is unknown, so just read it error free.
    device.interrupt_set


if __name__ == "__main__":

    print("setup...")
    setup_module()
    print("test_indoor_outdoor...")
    test_indoor_outdoor()
    print("power_down...")
    test_power_down()
    print("noise_floor_level...")
    test_noise_floor_level()
    print("watchdog...")
    test_watchdog()
    print("spike_rejection...")
    test_spike_rejection()
    print("strike_count_threshold...")
    test_strike_count_threshold()
    print("disturber_mask...")
    test_disturber_mask()
    print("freq_divisor...")
    test_freq_divisor()
    print("output_antenna_freq...")
    test_output_antenna_freq()
    print("output_srco...")
    test_output_srco()
    print("output_trco...")
    test_output_trco()
    print("tuning_capacitance...")
    test_tuning_capacitance()
    print("reset...")
    test_reset()
    print("commands_which_do_not_change_readable_values...")
    test_commands_which_do_not_change_readable_values()
    print("registers_with_unpredictable_states...")
    test_registers_with_unpredictable_states()
    print("Interrupt pin...")
    test_read_interrupt_pin()
    print("teardown...")
    teardown_module()
    print("Tests complete.")
