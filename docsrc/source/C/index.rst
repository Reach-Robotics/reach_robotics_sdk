BPL Protocol - C/C++ SDK
========================

Welcome to the BPL Protocol Documentation C/C++ SDK

Here you can find instructions on how to use the C/C++ SDK

You can find the SDK here: :download:`bplprotocol.h <../../../C/bplprotocol.h>`
Or in the SDK at `C/bplprotocol.h`

Enable use of the SDK
------------------------------
For enable full access to the SDK follow the following instructions.

SDK
--------------------------
.. toctree::
    :maxdepth: 1

    ../bplprotocol/sdk_install

Examples
----------------------
Some code examples using the bplprotocol can be found under C/examples

.. toctree::
    :maxdepth: 1

    examples/encoding_packets
    examples/decoding_packets

Running Examples
~~~~~~~~~~~~~~~~~~~~~~
To view the C examples compile them like so.
Replace the example name with the Relevant example you would like to test.


.. code-block:: bash

   cd ./C/examples/
   gcc decodePacketExample.c -I .. -o example_script

   # To run the example
   ./example_script