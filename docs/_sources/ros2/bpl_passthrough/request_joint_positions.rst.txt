Using bpl_passthrough to get joint positions
=========================================================================

This example demonstrates how to request read joint
positions from joints on a manipulator.

To launch this example run the launch file.

.. tabs::

    .. tab:: Serial

        .. code-block:: bash

            ros2 launch bpl_passthrough udp_passthrough_example.launch.py serial_port:="/dev/ttyUSB0"
            
    .. tab:: UDP

        .. code-block:: bash

            ros2 launch bpl_passthrough udp_passthrough_example.launch.py ip_address:=192.168.2.4 port:=6789

The example will request and printout the positions of a manipulators joints.

The script communicates the to passthrough node via the :code:`/tx` and :code:`/rx` topics.
It publishes request packets to the :code:`/tx` topic at a set frequency.
It subscribes the to :code:`/rx` topic and listens for positions packets.

.. note::
    This script has been tested to work at 400 Hz over a UDP Connection to the Base MCU.

.. literalinclude:: ../../../../ROS2/bpl_passthrough/bpl_passthrough/request_joint_positions.py

