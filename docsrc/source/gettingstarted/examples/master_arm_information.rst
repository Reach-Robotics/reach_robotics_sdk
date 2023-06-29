Master Arm Information
========================================

A master arm is intended to control an arm by passing through the byte stream without the need to decode.
In the case where you may wish the decode the information of the master arm this can be done so.
The following script provides an example of decoding the master arm to obtain joint states / velocities,
buttons states and the mode of the master arm.

.. note::
    The script will work for 7 FN Manipulators Configured for a Bravo 7 FN. Do this via reach control before running this script.


The wrist can be controlled by the joint on the master arm or by the joystick but not both at the same time.
This is configured on reach control.


.. literalinclude:: ../../../../bplprotocol/examples/ma_information.py
