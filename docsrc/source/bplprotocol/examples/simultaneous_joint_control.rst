Controlling Joints Simultaneously
========================================

Joints Control packets must be send individually to the joints.
If you would like to simultaneously control the joints send the control packets in one after another.


Alternatively you can concatenate all the packets to form list of bytes then proceed to send all the packets out at once.


The following is an example of doing this.


.. tabs::

   .. tab:: Serial

      .. literalinclude:: ../../../../bplprotocol/examples/simultaneous_joint_control_serial.py


   .. tab:: UDP

      .. literalinclude:: ../../../../bplprotocol/examples/simultaneous_joint_control_eth.py



