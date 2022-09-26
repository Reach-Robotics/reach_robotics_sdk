ROS
================================================

Here are details for usage of the Blueprint lab ROS1 Packages

Installation
---------------------
To install ROS please follow instructions at http://wiki.ros.org/ROS/Installation.

First clone the repository to your workspace

.. code-block:: bash

   cd ~/catkin_ws/src
   git clone https://github.com/blueprint-lab/Blueprint_Lab_Software.git

Now install the bplprotocol python package

.. code-block:: bash

   cd Blueprint_Lab_Software/bplprotocol
   pip install .

To build all packages

.. code-block:: bash

   cd ~/catkin_ws
   catkin_make

Usage
---------------------------
The ROS Folder is split into several ROS packages.

bpl_passthrough
^^^^^^^^^^^^^^^^^^^^^^^^
The BPL Passthrough is the core package that allows communication to bpl products.
You can connect to a manipulator via serial or UDP (Bravo arms only).
The :code:`bpl_msgs/Packet` data field is structured as a list of uint8. This is a list of bytes.
For incoming floats, they will be encoded as 4 bytes. Refer to the bplprotocol SDK on how to decode these bytes into floats.

.. code-block:: bash

   roslaunch bpl_passthrough udp_passthrough.launch

or

.. code-block:: bash

   roslaunch bpl_passthrough serial_passthrough.launch

Published Topics
""""""""""""""""""""""
:code:`/rx` (:code:`bpl_msgs/Packet`) - Received Packets from the manipulator


Subscribed Topics
""""""""""""""""""""""
:code:`/tx` (:code:`bpl_msgs/Packet`) - Packets that will be sent to the manipulator

Parameters - udp_passthrough.py
"""""""""""""""""""""""""""""""""""""""""""""""""

:code:`ip_address` (string) - IP Address of the arm. (Defaults to 192.168.2.3)

:code:`port` (int) - UDP Port of the arm. (Defaults to 6789)


Parameters - serial_passthrough.py
"""""""""""""""""""""""""""""""""""""""""""""""""
:code:`serial_port` (string) - Serial Port to connect to the arm (Defaults to "/dev/ttyUSB0")

:code:`baudrate` (int) - Baudrate port of the serial connection. (Defaults to 115200)

Examples
"""""""""""""""""""""""""""""""""""""""""""""""""
.. toctree::
    :maxdepth: 3

    bpl_passthrough/request_joint_positions

bpl_alpha_description
^^^^^^^^^^^^^^^^^^^^^^^^

The BPL Alpha Description package contains the Universal Robot Description File (URDF) files of the alpha range of manipulators.

Supported Products:

- RA-5001 - Reach Alpha 5

To view an Alpha 5 URDF:

.. code-block:: bash

   roslaunch bpl_alpha_description view_urdf.launch


bpl_bravo_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The BPL Bravo Description package contains the Universal Robot Description File (URDF) files for the bravo range of manipulators.

Supported Products:

- RB-7001 - Reach Bravo 7
- RB-5001 - Reach Bravo 5

To view an Bravo 7 URDF:

.. code-block:: bash

    roslaunch bpl_bravo_description view_bravo_7.launch

To view an Bravo 5 URDF:

.. code-block:: bash

    roslaunch bpl_bravo_description view_bravo_5.launch

bpl_bravo_description_mk2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The BPL Bravo Description MK2 package contains the Universal Robot Description File (URDF) files for the bravo mk2 range of manipulators.

Supported Products:

- RB-5001 - Reach Bravo 5 MK2
- RB-7001 - Reach Bravo 7 MK2
- RB-7201 - Reach Bravo 7 MK2 with FT sensor

To view an Bravo 5 URDF:

.. code-block:: bash

    roslaunch bpl_bravo_description_mk2 view_bravo_5.launch

To view an Bravo 7 URDF:

.. code-block:: bash

    roslaunch bpl_bravo_description_mk2 view_bravo_7.launch

To view an Bravo 7 with FT URDF:

.. code-block:: bash

    roslaunch bpl_bravo_description_mk2 view_bravo_7_ft.launch
