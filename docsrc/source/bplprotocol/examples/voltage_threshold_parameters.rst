Voltage Threshold Parameters - Auto Stow on Voltage Change
==================================================================

Bravo 5 and 7 function manipulators have the ability to automatically stow and disable themselves when a voltage change is detected.


.. note::
    This feature only is available for Bravo 5 or 7 Function Manipulators.
    Requires TX2 version `0.1.9` and 708 version `0.3.4` or above.


If enabled, upon being within a voltage margin for a set period of time the arm will automatically move to a set stow position.

The stow position is configured as the set position preset for `Position Preset 0`.
For more information on Position Presets see the Reach System Communication Protocol Document in :doc:`../../../documentation/index`.

Usage
---------------

Use Packet VOLTAGE_THRESHOLD_PARAMETERS (0x99) to configure the settings.

The packet consists of 4 floats.

Enabled: Set to 0 to disable, 1 to enable

Voltage Min (V): Minimum Voltage to trigger the auto-stow.

Voltage Max (V): Maximum Voltage to trigger the auto-stow.

Time (seconds): Time required in the voltage margin to trigger the auto-stow.

Send and Save this packet to the 708 (Device ID: 0x0D).


Example
--------------------

To set the parameters

.. literalinclude:: ../../../../bplprotocol/examples/voltage_threshold_parameters.py
    :lines: 1-25

To ensure that they are saved after a reboot

.. literalinclude:: ../../../../bplprotocol/examples/voltage_threshold_parameters.py
    :lines: 25 - 38