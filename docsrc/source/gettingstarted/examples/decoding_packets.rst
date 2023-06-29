Decoding Packets
=======================================================================
Example on how to decode a list of bytes into a packet with a Device ID, Packet ID and corresponding Data.

Decoding a Packet
-----------------------------------------------------------------------
An example on how to decode a list of bytes into a packet. The data is expected to come in as a list of unsigned 8 bit intergers.



.. literalinclude:: ../../../../C/examples/decodePacketExample.c
   :language: c


Decoding a Packet with a Single Float
-----------------------------------------------------------------------
An example on how to decode a list of bytes into a packet. The data is expected to come in as a single float represented as 4 bytes. 
This example will show you how to convert the 4 bytes into a single float. This is a very common form for BPL Packets like Position and Velocity.

.. literalinclude:: ../../../../C/examples/decodePacketWithFloatExample.c
    :language: c


Decoding a Packet with a list of Floats
-----------------------------------------------------------------------
Some packets will contain a list of floats in their data. This example will decode that packet into a list of floats.

.. literalinclude:: ../../../../C/examples/decodePacketWithFloatListExample.c
    :language: c