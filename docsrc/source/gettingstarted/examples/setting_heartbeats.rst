Setting Heartbeats
========================================

Joints can be set to publish data at a set frequency using heartbeat packets.

Up to 10 packets can be set a a heartbeat per device.
Frequency can be set to a whole number between 0 - 255 Hz.


.. warning::
    Devices have been tested to support a maximum of 4 heartbeat packets at 255 Hz. Behaviour is undefined for values
    greater than this.


The following is an example of setting the heartbeat over a UDP Connection.

.. literalinclude:: ../../../../bplprotocol/examples/set_heartbeat.py
    :lines: 1-35

The set heartbeat packets are sent automatically at the specified frequency. They can be read by listening to the UDP Socket.

.. literalinclude:: ../../../../bplprotocol/examples/set_heartbeat.py
    :lines: 36-52
