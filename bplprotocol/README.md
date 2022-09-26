# BPL Protocol
The BPL Protocol is a package that allows for the usage of the Reach Robotics Communication Protocol.

## Installation
Use pip to install
```
 Insert install command here
pip install .
```

## Usage
```python
from bplprotocol import BPLProtocol, PacketID

# Encode a velocity command packet:
# velocity of +3.0 mm/s to device 0x01 (Jaws)
data_bytes = BPLProtocol.encode_packet(0x01, PacketID.VELOCITY, BPLProtocol.encode_floats([3.0]))
```

## Support

For support please contat support@reachrobotics.com