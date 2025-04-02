.. _getting_started:
    
Getting Started
========================

To get started with the Reach Robotics SDK, you will first need to clone the SDK into
your development workspace.

.. code-block:: bash

   cd ~/workspace/src
   git clone https://github.com/Reach-Robotics/reach_robotics_sdk.git


The SDK is available for both Python and C/C++ and includes a ROS 2 node (communication bridge) 
for easy integration with ROS 2 applications. If you intend to use the ROS 2 node, ensure the 
repository is cloned into your ROS workspace src folder. 

.. note::
    The Reach System communication protocol is only compiled/tested on the following 
    platform/operating system combinations:

    * aarch-64 (Linux)
    * x86-64 (Linux & Windows)

Getting Started With Python
---------------------------

To get started with the Reach System Python protocol, install the ``rs_protocol`` package and 
requirements using the provided setup script for your operating system.

**Windows:**

.. code-block:: bash

    ./scripts/setup_windows.bat

**Linux:**

.. code-block:: bash
    
    ./scripts/setup_linux.sh

Once the setup script has been executed, you can run the
examples provided in the  ``rs_protocol/examples`` folder.

.. code-block:: python

   cd ./rs_protocol/examples/
   python3 basic_actuator_control.py


.. note::

    The minimum supported Python version for this package is 3.6. 

Getting Started With C/C++
---------------------------

To get started with the Reach System C/C++ communications protocol, navigate to the src folder.

.. code-block:: none

    reach_robotics_sdk
    └── rs_protocol
        └── src
            └── examples
                ├── decodePacketExample.c
                └── encodePacketExample.c
            └── packetID.h 
            └── rs_protocol.h

In the src folder you will find the packetID.h file as well as the rs_protocol.h file. These files 
may be included in your c/c++ project to decode and encode Reach System packets. Some examples of
decoding and encoding packets are provided in the examples folder. You can compile and execute 
examples using the GCC toolchain as shown below. 

.. code-block:: bash

   cd ./rs_protocol/srs/examples/
   gcc decodePacketExample.c -I .. -o decode_example.o

   # To run the example
   ./decode_example.o