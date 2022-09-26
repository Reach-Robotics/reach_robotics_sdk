Accessing the Force Torque Sensor
=============================================

.. image:: Force-Torque-Sensor-1.jpg
    :width: 600

The Force Torque Sensor is a bravo accessory that allows for the detection of force and torque applied.

X and Y axis are labeled on the Sensor. The Z axis is along the axis of the sensor.


Taring of the sensor is achieved by sending an ATI_FT Packet device 0x0D

.. literalinclude:: ../../../../bplprotocol/examples/ft_sensor.py
    :lines: 1-30



Reading of the FT Sensor can be done by requesting the ATI_FT Packet from device 0x0D

.. literalinclude:: ../../../../bplprotocol/examples/ft_sensor.py
    :lines: 32 - 58


.. note::
    In testing the force torque sensor was able to be read at a rate of approximately 450 - 490 Hz.
    This was with a UDP Connection to the Base Board.