# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT

# Many Pylnt conventions are broken for the sake of test readability
# Others fail because Pylint doesn't understand Pytest.
# Therefore skip this file.
# pylint: skip-file

import pytest
from CircuitPython_AS3935 import biffobear_as3935 as as3935


@pytest.mark.parametrize(
    "spi, cs_pin_in, cs_pin_out, baud, spibus, int_pin",
    [
        (
            "spi1",  # The SPI bus instance, arg for as3935_spi
            "cs_pin_in1",  # The chip select pin (e.g. board.D5), arg for as3935_spi
            "cs_pin_out1",  # The DigitalInOut object used in call to SPIDevice
            "baud1",  # Baudrate optional arg for as3935_spi
            "SPIbus1",  # SPIDevice used as arg for as3935_spi passed to AS3935
            "int_pin1",  # Interrupt pin (e.g. board.D7), arg for
        ),
        (
            "spi2",
            "cs_pin_in3",
            "cs_pin_out2",
            "baud1",
            "SPIbus2",
            "int_pin2",
        ),
    ],
)
def test_as3935_instantiated_with_correct_args_from_as3935_spi(
    mocker, spi, cs_pin_in, cs_pin_out, baud, spibus, int_pin):
    assert issubclass(as3935.AS3935_SPI, as3935.AS3935)
    mock_cs_pin = mocker.Mock(name=cs_pin_in)
    mock_digitalio = mocker.patch.object(
        as3935.digitalio, "DigitalInOut", return_value=cs_pin_out
    )
    mock_spidevice = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", return_value=spibus
    )
    mock_as3935_init = mocker.patch.object(as3935.AS3935, "__init__", autospec=True, return_value=None)
    as3935.AS3935_SPI(spi, mock_cs_pin, baud, interrupt_pin=int_pin)
    # Check that cs pin converted to a DigitalInOut object
    mock_digitalio.assert_called_once_with(mock_cs_pin)
    # Confirm SPIDevice called with correct values
    mock_spidevice.assert_called_once_with(
        spi, cs_pin_out, baudrate=baud, polarity=1, phase=0
    )
    mocker.resetall()
    test_as3935 = as3935.AS3935_SPI(spi, mock_cs_pin, interrupt_pin=int_pin)
    # Confirm that SPIDevice is called with a default baudreate
    default_spi_baudrate = 1_000_000
    mock_spidevice.assert_called_once_with(
        spi, cs_pin_out, baudrate=default_spi_baudrate, polarity=1, phase=0
    )
    # Check that self._bus is correctly assigned
    assert test_as3935._bus == spibus
    # Check that AS3935 instantiated with correct args
    mock_as3935_init.assert_called_once_with(test_as3935, interrupt_pin=int_pin)
