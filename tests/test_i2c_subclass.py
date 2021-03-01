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
def test_as3935_AS3935_I2C_instantiated_with_correct_args_from_as3935_i2c(
    mocker, i2c, i2cbus, address, int_pin, as3935_inst
):
    assert issubclass(as3935.AS3935_I2C, as3935.AS3935)
    mock_i2cdevice = mocker.patch.object(
        as3935.i2c_dev, "I2CDevice", return_value=i2cbus
    )
    mock_as3935 = mocker.patch.object(as3935, "AS3935", return_value=as3935_inst)
    as3935.AS3935_I2C(i2c, address, interrupt_pin=int_pin)
    # Confirm that I2CDevice called with correct values
    mock_i2cdevice.assert_called_once_with(i2c, address)
    mocker.resetall()
    # Confirm I2CDevice is called with the correct default address
    result = as3935.AS3935_I2C(i2c, interrupt_pin=int_pin)
    default_i2c_address = 0x03
    mock_i2cdevice.assert_called_once_with(i2c, 0x03)
    assert result.bus == i2cbus
    # Check that AS3935 instantiated with correct args
    # mock_as3935.assert_called_once_with(bus=i2cbus, interrupt_pin=int_pin)
    # Confirm an instance of AS3935 is returned
    # assert result == as3935_inst
