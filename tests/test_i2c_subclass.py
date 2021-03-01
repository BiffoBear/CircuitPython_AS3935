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
    "i2c, i2cbus, address, int_pin, as3935_inst",
    [("i2c1", "i2cbus1", "address1", "int_pin1", "as3935_inst")],
)
def test_as3935_AS3935_I2C_instantiated_with_correct_args(
    mocker, i2c, i2cbus, address, int_pin, as3935_inst
):
    assert issubclass(as3935.AS3935_I2C, as3935.AS3935)
    mock_i2cdevice = mocker.patch.object(
        as3935.i2c_dev, "I2CDevice", return_value=i2cbus
    )
    mock_as3935_init = mocker.patch.object(as3935.AS3935, "__init__", return_value=None)
    as3935.AS3935_I2C(i2c, address, interrupt_pin=int_pin)
    # Confirm that I2CDevice called with correct values
    mock_i2cdevice.assert_called_once_with(i2c, address)
    mocker.resetall()
    # Confirm I2CDevice is called with the correct default address
    test_as3935_i2c = as3935.AS3935_I2C(i2c, interrupt_pin=int_pin)
    default_i2c_address = 0x03
    mock_i2cdevice.assert_called_once_with(i2c, 0x03)
    # Confirm that the I2CDevice is assighed to self._bus
    assert test_as3935_i2c._bus == i2cbus
    # Check that AS3935 instantiated with correct args
    mock_as3935_init.assert_called_once_with(interrupt_pin=int_pin)

@pytest.mark.parametrize("i2c", [("i2c1",)])
def test_write_byte_out_calls_i2c_dev_write_with_correct_kwargs(mocker, i2c):
    # Confirm that the correct _write_byte_out is being called
    assert as3935.AS3935_I2C._write_byte_out.__qualname__ == "AS3935_I2C._write_byte_out"
    mock_as3935_init = mocker.patch.object(as3935.AS3935, "__init__", return_value=None)
    mock_i2cdevice = mocker.patch.object(as3935.i2c_dev, "I2CDevice", autospec=True, return_value=mocker.MagicMock())
    # mock_as3935_init = mocker.patch.object(as3935.AS3935, "__init__", return_value=None)

    test_register = as3935._Register(0x0F, 0x55, 0x00)
    test_as3935_i2c = as3935.AS3935_I2C(i2c, interrupt_pin="int_pin")
    # test_as3935_i2c._bus.writeto.return_value = None
    test_as3935_i2c._write_byte_out(test_register, 0x22)
    test_as3935_i2c._bus.writeto.assert_called_once_with(0x0f, bytearray([0x22, 0x00]), end=1)
