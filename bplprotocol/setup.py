from distutils.core import setup
from genericpath import isfile
import sys
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
packetID_path = os.path.join(dir_path, "bplprotocol/packetID.py")
if not os.path.isfile(packetID_path):
    raise ImportError("Please install the Reach Robotics SDK Critical Components.")

setup(
    name='bplprotocol',
    version='1.0.0',
    packages=['bplprotocol'],
    install_requires=['cobs', 'crcmod'],
    long_description=open('README.md').read(),
)