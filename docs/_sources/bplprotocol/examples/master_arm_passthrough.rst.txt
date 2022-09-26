Master Arm Passthrough
========================================

Communications from a configured Master Arm can be passed through to a manipulator to allow for control of the arm. 
The Master arm communicates on a serial port and can be easily forwarded to your arms connection (Serial or UDP). 

The following is an example of setting the passing a master arm serial connection to a manipulator connected via serial.

.. literalinclude:: ../../../../bplprotocol/examples/master_arm_passthrough_serial.py


