Introduction
============

.. image:: https://readthedocs.org/projects/biffobear-circuitpython-as3935/badge/?version=latest
    :target: https://circuitpython-as3935.readthedocs.io/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/BiffoBear/Biffobear_CircuitPython_AS3935/workflows/Build%20CI/badge.svg
    :target: https://github.com/BiffoBear/Biffobear_CircuitPython_AS3935/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython driver library for the AS3935 lightning detector.

.. warning:: The AS3935 chip supports I2C but Sparkfun found it unreliable so
   this driver is SPI only.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
    as a standard element. Stay tuned for PyPI availability!

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-as3935/>`_.
To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-as3935

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-as3935

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-as3935


Running The Unittests
=====================

To run the unittests on you will need to install Pytest and Pytest Mock.

.. code-block:: shell

    pip3 install pytest pytest-mock

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install pytest pytest-mock


Usage Example
=============

.. code-block:: python
    
    import time
    import board
    import biffobear_as3935
    
    spi = board.SPI  # Works for most Adafruit and Blinka boards.
    # Edit the pins to match the connections to your board.
    cs_pin = board  # Connect to the sensor chip select pin.
    interrupt_pin = board(D7)  # Connected to the sensor interrupt pin.
    
    sensor = biffo_bear_as3935(spi, cs_pin, interrupt_pin=interrupt_pin)
    
    while True:
        if sensor.interrupt_set:  # An event has occurred
            # The interrupt_status is cleared after a read, so assign it
            # to a variable in case you need the value later.
            event_type = sensor.interrupt_status == sensor.LIGHTNING
            if event_type == 0x08  # It's a lightning event
                print(f"Strike Energy = {sensor.energy})
                print(f"Distance to storm front = {sensor.distance} km")
            elif event_type == sensor.DISTURBER:
                print("False alarm")
        # Minimum time between strike events is 1 second so poll frequently!
        time.sleep(0.5)
        


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/BiffoBear/Biffobear_CircuitPython_AS3935/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
