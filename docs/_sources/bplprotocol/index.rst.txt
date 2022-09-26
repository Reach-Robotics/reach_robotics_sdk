BPL Protocol
========================

Welcome to the BPL Protocol Documentation

Enable use of the SDK
------------------------------
For enable full access to the SDK follow the following instructions.

SDK
--------------------------
.. toctree::
    :maxdepth: 1

    sdk_install

Python Installation
-------------------

Here are instructions to install for python

.. code-block:: bash

   cd ./bplprotocol
   pip3 install .

Usage
-----------------------

.. code-block:: python

    from bplprotocol import BPLProtocol, PacketID

    # Encode a velocity command packet:

    # Encodes the packet with a \x00 on the end (null byte)

    data_bytes = BPLProtocol.encode_packet(0x01, PacketID.VELOCITY, BPLProtocol.encode_floats([3.0]))     # velocity of +3.0 mm/s to device 0x01 (Jaws)


    # Appending bytes together  #
    data_bytes += BPLProtocol.encode_packet(0x02, PacketID.REQUEST, [PacketID.VELOCITY, PacketID.POSITION]))  # Request packet of Velocity and Position from the Wrist (device_id 0x02)



    # Decoding any read bytes.
    # Option 1 Splitting and decoding manually.

    # Split the packet at with \x00 (null byte) as the seperator. incomplete_bytes contains bytes that have not seen \x00 to terminate it
    read_bytes, incomplete_bytes = BPLProtocol.packet_splitter(bytes)

    # Decode the bytes into a list of packets
    packets = BPLProtocol.parse_packet(read_bytes)

    # Option 2, Reading packets from the packet reader

    pr = PacketReader()

    # The packet reader will decode packets, and reconstruct incomplete packets
    packets = pr.receive_bytes(data_bytes)

    # Each decoded packet is a 3 element tuple of device_id, packet_id, and data
    for packet in packets:
        # int, int, bytes
        device_id, packet_id, data = packet


        # A position packet is a float packet. Float packets must be decoded

        if packet_id == PacketID.POSITION:
            float_list = packet_id.decode_floats(data)

            # Position packets contain 1 float, other packets may contain more than one float.
            position = float_list[0]

            print(f"I received a position packet from device: {device_id} with position: {position})

        else:
            # Non float packets can be decoded as list of integers
            data_list = list(data)
            print(f"I received packet from device {device_id} with packet id: {packet_id} and {data_list})


Examples
----------------------
Some code examples using the bplprotocol can be found under bplprotocol/examples

.. toctree::
    :maxdepth: 3

    examples/requesting_joint_positions
    examples/setting_heartbeats
    examples/master_arm_passthrough
    examples/master_arm_information
    examples/force_torque_sensor
    examples/joint_control
    examples/simultaneous_joint_control
    examples/voltage_threshold_parameters


SDK
--------------------------
.. toctree::
    :maxdepth: 3

    sdk