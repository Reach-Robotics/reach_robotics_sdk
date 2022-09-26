from distutils.core import setup
from genericpath import isfile
import sys
import os

if sys.version_info[0] == 3:

    dir_path = os.path.dirname(os.path.realpath(__file__))
    packetID_path = os.path.join(dir_path, "bplprotocol/packetID.py")
    if not os.path.isfile(packetID_path):
        raise ImportError("Please install the Reach Robotics SDK Critical Components. See https://reach-robotics.github.io/reach_robotics_sdk/bplprotocol/sdk_install.html")
    
    setup(
        name='bplprotocol',
        version='1.0.0',
        packages=['bplprotocol'],
        install_requires=['cobs', 'crcmod'],
        long_description=open('README.md').read(),
    )
else:

    dir_path = os.path.dirname(os.path.realpath(__file__))
    packetID_path = os.path.join(dir_path, "src27/bplprotocol/packetID.py")
    if not os.path.isfile(packetID_path):
        raise ImportError("Please install the Reach Robotics SDK Critical Components. See https://reach-robotics.github.io/reach_robotics_sdk/bplprotocol/sdk_install.html")
    
    setup(
        name='bplprotocol',
        version='1.0.0',
        package_dir={"": 'src27'},
        packages=['bplprotocol'],
        install_requires=['cobs', 'crcmod'],
        long_description=open('README.md').read(),
    )