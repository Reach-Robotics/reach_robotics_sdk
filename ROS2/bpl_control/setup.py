from setuptools import setup

package_name = 'bpl_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnsumskas',
    maintainer_email='john@sumskas.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = bpl_control.joint_state_publisher:main',
            'control_node = bpl_control.control_node:main',
            'end_effector_pose_publisher = bpl_control.end_effector_pose_publisher:main'
            # 'b7_gui = bpl_control.b7_gui:main',
            # 'b5_gui = bpl_control.b5_gui:main'
        ],
    },
)
