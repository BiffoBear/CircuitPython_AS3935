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
    mocker, spi, cs_pin_in, cs_pin_out, baud, spibus, int_pin
):
    assert issubclass(as3935.AS3935, as3935._AS3935)
    mock_cs_pin = mocker.Mock(name=cs_pin_in)
    mock_digitalio = mocker.patch.object(
        as3935.digitalio, "DigitalInOut", return_value=cs_pin_out
    )
    mock_spidevice = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", return_value=spibus
    )
    mock_as3935_init = mocker.patch.object(
        as3935._AS3935, "__init__", autospec=True, return_value=None
    )
    as3935.AS3935(spi, mock_cs_pin, baud, interrupt_pin=int_pin)
    # Check that cs pin converted to a DigitalInOut object
    mock_digitalio.assert_called_once_with(mock_cs_pin)
    # Confirm SPIDevice called with correct values
    mock_spidevice.assert_called_once_with(
        spi, cs_pin_out, baudrate=baud, polarity=1, phase=0
    )
    mocker.resetall()
    test_as3935 = as3935.AS3935(spi, mock_cs_pin, interrupt_pin=int_pin)
    # Confirm that SPIDevice is called with a default baudreate
    default_spi_baudrate = 1_000_000
    mock_spidevice.assert_called_once_with(
        spi, cs_pin_out, baudrate=default_spi_baudrate, polarity=1, phase=0
    )
    # Check that self._bus is correctly assigned
    assert test_as3935._bus == spibus
    # Check that AS3935 instantiated with correct args
    mock_as3935_init.assert_called_once_with(test_as3935, interrupt_pin=int_pin)


@pytest.mark.parametrize(
    "addr, data_byte, buffer",
    [(0x0F, 0xFF, 0x0F), (0x3F, 0x00, 0x3F), (0xF0, 0x55, 0x30)],
)
def test_write_byte_out_sets_correct_bits_for_write_address_and_sends_correect_data(
    mocker, addr, data_byte, buffer
):
    mock_sleep = mocker.patch.object(as3935.time, "sleep", autospec=True)
    mocker.patch.object(as3935.digitalio, "DigitalInOut")
    mock_as3935_init = mocker.patch.object(
        as3935._AS3935, "__init__", return_value=None
    )
    mock_spidevice = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", autospec=True, return_value=mocker.MagicMock()
    )
    test_register = as3935._Register(addr, 0x55, 0x00)
    test_as3935_spi = as3935.AS3935("spi", "cs_pin", interrupt_pin="int_pin")
    test_as3935_spi._write_byte_out(test_register, data_byte)
    # Check that SPIDevice.write is called with the write bits set in the register address
    assert as3935._BUFFER[0] == buffer
    name, args, kwargs = test_as3935_spi._bus.__enter__.return_value.mock_calls[0]
    assert name == "write"
    assert args == (as3935._BUFFER,)
    assert kwargs == {"end": 2}
    mock_sleep.assert_called_once_with(0.01)


@pytest.mark.parametrize(
    "addr, data_byte, buffer",
    [(0x0F, 0xFF, 0x4F), (0x3F, 0x00, 0x7F), (0xF0, 0x55, 0x70)],
)
def test_read_byte_in_sets_correct_bits_for_read_address_and_sends_correect_data(
    mocker, addr, data_byte, buffer
):
    mock_sleep = mocker.patch.object(as3935.time, "sleep", autospec=True)
    mocker.patch.object(as3935.digitalio, "DigitalInOut")
    mock_as3935_init = mocker.patch.object(
        as3935._AS3935, "__init__", return_value=None
    )
    mock_spidevice = mocker.patch.object(
        as3935.spi_dev, "SPIDevice", autospec=True, return_value=mocker.MagicMock()
    )
    test_register = as3935._Register(addr, 0x55, 0x00)
    test_as3935_spi = as3935.AS3935("spi", "cs_pin", interrupt_pin="int_pin")
    test_as3935_spi._read_byte_in(test_register)
    name, args, kwargs = test_as3935_spi._bus.__enter__.return_value.mock_calls[0]
    assert name == "write"
    assert kwargs == {"end": 1}
    name, _, kwargs = test_as3935_spi._bus.__enter__.return_value.mock_calls[1]
    assert name == "readinto"
    assert kwargs == {"end": 1}
    mock_sleep.assert_called_once_with(0.01)
