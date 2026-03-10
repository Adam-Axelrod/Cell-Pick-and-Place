"""
bota_driver
===========

Python bindings for the BotaDriver C++ library.

Classes:
--------
BotaDriver:
    Main driver class for device interaction.

    Methods:
        __init__(config_path: str)
            Initialize the driver with a configuration file path.

        configure()
            Configure the driver

        activate()
            Activate the driver and start data acquisition.

        deactivate()
            Deactivate the driver, stopping data acquisition.

        shutdown()
            Shut down the driver and release resources.

        cleanup()
            Clean up the driver state.

        get_driver_state() -> DriverState
            Get the current state of the driver.

        get_driver_version_string() -> str
            Get the version string of the underlying driver.

        read_frame() -> BotaFrame
            Read a single data frame (non-blocking).

        read_frame_blocking() -> BotaFrame
            Read a single data frame (blocking until available).

        tare()
            Zero the force/torque sensor.

Enums:
------
DriverState:
    Enum representing the state of the driver.

"""

from .bota_driver_ext import __doc__, DriverState, BotaDriver

