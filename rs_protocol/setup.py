import os
from distutils.core import setup

setup(
    name='rs_protocol',
    version='2.0.0',
    packages=['rs_protocol'],
    install_requires=['cobs', 'crcmod'],
    package_data={
        "rs_protocol": ["../lib/*.dll", "../lib/*.dll.a", "../lib/*.so"],
    },
)