from setuptools import setup
import os
from glob import glob

package_name = 'bpl_passthrough'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsumskasbpl',
    maintainer_email='j.sumskas@reachrobotics.com',
    description='Package that allows for communication to BPL Products.',
    license='Properitary Software, No Public License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_passthrough = bpl_passthrough.serial_passthrough:main',
            'udp_passthrough = bpl_passthrough.udp_passthrough:main',
            'request_joint_positions = bpl_passthrough.request_joint_positions:main',
            # 'request_km_end_pos = scripts.request_km_end_pos:main',
        ],
    },
)
