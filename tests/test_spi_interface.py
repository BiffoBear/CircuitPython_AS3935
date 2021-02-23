# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT

# Many Pylnt conventions are broken for the sake of test readability
# Others fail because Pylint doesn't understand Pytest.
# Therefore skip this file.
# pylint: skip-file

import time
from collections import namedtuple
from random import random
from unittest.mock import PropertyMock
import pytest
from CircuitPython_AS3935 import biffobear_as3935 as as3935


def test_as3925_spi_init_called_with_correct_args_and_kwargs(mocker):
    mock_init = mocker.patch.object(
        as3935.AS3935_SPI, "__init__", autospec=True, return_value=None
    )
    # Check that the correct class is subclassed
    assert issubclass(as3935.AS3935_SPI, as3935.AS3935)
    # Check the args and kwargs with default baudrate
    test_class = as3935.AS3935_SPI("spi", "cs", interrupt_pin="pin")
    mock_init.assert_called_once_with(test_class, "spi", "cs", interrupt_pin="pin")
    # Check the args and kwargs with assigned baudrate
    mock_init.reset_mock()
    test_class = as3935.AS3935_SPI("spi", "cs", baudrate=1_000_000, interrupt_pin="pin")
    mock_init.assert_called_once_with(
        test_class, "spi", "cs", baudrate=1_000_000, interrupt_pin="pin"
    )


@pytest.mark.parametrize(
    "spi, cs, cs_pin, interrupt_pin",
    [("spi1", "cs1", "cs_pin1", "int1"), ("spi2", "cs2", "cs_pin2", "int2")],
)
def test_calls_from_init_made_with_correct_args_and_kwargs(
    mocker, spi, cs, cs_pin, interrupt_pin
):
    mock_spi_device = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", autospec=True, return_value="SPIDevice"
    )
    mock_digtalio_digitalinout = mocker.patch.object(
        as3935.digitalio, "DigitalInOut", autospec=True, return_value=cs_pin
    )
    mock_super_init = mocker.patch.object(
        as3935.AS3935, "__init__", autospec=True, return_value=None
    )
    # Test call to SPIDevice
    test_class = as3935.AS3935_SPI(spi, cs, interrupt_pin=interrupt_pin)
    mock_spi_device.assert_called_once_with(
        spi, cs_pin, baudrate=2_000_000, polarity=1, phase=0
    )
    # Check that digitalio is called with the correct pin to set the chip select
    mock_digtalio_digitalinout.assert_called_once_with(cs)
    # Also test that _device is set to the SPIDevice
    assert test_class._device == "SPIDevice"
    # Check that super.__init__ is called with the interrupt pin
    mock_super_init.assert_called_once_with(test_class, interrupt_pin=interrupt_pin)


def test_spi_device_called_with_correct_baudrates(mocker):
    mock_spi_device = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", autospec=True, return_value="SPI_Device"
    )
    mock_digtalio_digitalinout = mocker.patch.object(
        as3935.digitalio, "DigitalInOut", autospec=True, return_value="cs"
    )
    mock_super_init = mocker.patch.object(
        as3935.AS3935, "__init__", autospec=True, return_value=None
    )
    # Confirm SPIDevice is called with the default baudrate
    test_class = as3935.AS3935_SPI("spi", "cs", interrupt_pin="pin")
    mock_spi_device.assert_called_once_with(
        "spi", "cs", baudrate=2_000_000, polarity=1, phase=0
    )
    # Confirm SPIDevice is called with the baudrate passed to __init__
    mock_spi_device.reset_mock()
    test_class = as3935.AS3935_SPI("spi", "cs", baudrate=1_000_000, interrupt_pin="pin")
    mock_spi_device.assert_called_once_with(
        "spi", "cs", baudrate=1_000_000, polarity=1, phase=0
    )


# def test_read_byte_in_calls_spi_dev_write_with_correct_arguments(test_device, test_register):
#     test_device._read_byte_in(test_register)
#     # Complex mocking to work with "with x as y" constructs
#     name, _, kwargs = test_device._device.__enter__.return_value.mock_calls[0]
#     assert name == "write"
#     assert kwargs == {"end": 1}
#
#
# def test_read_byte_in_calls_spi_dev_readinto_with_correct_kwargs(
#     test_device, test_register
# ):
#     test_device._read_byte_in(test_register)
#     name, _, kwargs = test_device._device.__enter__.return_value.mock_calls[1]
#     assert name == "readinto"
#     assert kwargs == {"end": 1}
