# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT

# Many Pylnt conventions are broken for the sake of test readability
# Others fail because Pylint doesn't understand Pytest.
# Therefore skip this file.
# pylint: skip-file

import pytest
from CircuitPython_AS3935 import biffobear_as3935 as as3935


def test_spi_setup_function_called_with_correct_args(mocker):
    # Confirm that fucntion has correct calls
    as3935.as3935_spi("spi", "cs", "baudrate", interrupt_pin="pin")
    # Confirm that function can be called with a default baudrate
    as3935.as3935_spi("spi", "cs", interrupt_pin="pin")
