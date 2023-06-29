Requesting Joint Positions
========================================

A devices information can be requested by either individually requesting the joint via its device id,
or by requesting information from all devices using a broadcast 0xFF.

This examples shows how to efficiently request a joints positions using Broadcast Requests.

.. note::
    In testing an arms joint positions was able to be requested at 400 Hz over a UDP Connection to the Bravo Base Board.

.. literalinclude:: ../../../../bplprotocol/examples/requesting_joint_positions_udp.py
