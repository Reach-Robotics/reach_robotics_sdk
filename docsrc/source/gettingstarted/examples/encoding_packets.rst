Encoding Packets
=======================================================================
Example on how to encode a packet into bytes using the BPL C SDK.

Encoding a Packet
-----------------------------------------------------------------------
An example on how to encode a BPL Packet. BPL Packet consists of a Device ID (1-255), a PacketID (1-255), and an array of data (bytes)

.. literalinclude:: ../../../../C/examples/encodePacketBareExample.c
   :language: c

Encoding a Packet using the Packet Struct
-----------------------------------------------------------------------

.. literalinclude:: ../../../../C/examples/encodePacketExample.c
   :language: c


Encoding a Packet with Float Data
-------------------------------------------------------------------------

.. literalinclude:: ../../../../C/examples/encodePacketWithFloatExample.c
   :language: c


Encoding a Packet with a List of Floats Data
-------------------------------------------------------------------------

.. literalinclude:: ../../../../C/examples/encodePacketWithFloatListExample.c
   :language: c