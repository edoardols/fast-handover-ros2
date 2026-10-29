import os
from glob import glob
from setuptools import setup

package_name = 'smach_handover_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Turco',
    maintainer_email='enrico.turco@iit.it',
    description='SMACH State Machine and Action Servers for fast human-to-robot handover in ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_server = smach_handover_ctrl.controller_server:main',
            'planner = smach_handover_ctrl.planner:main',
        ],
    },
)