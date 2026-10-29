from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'handover_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Turco',
    maintainer_email='enrico.turco@iit.it',
    description='Vision and perception pipeline for fast human-to-robot handover in ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hand_detector = handover_vision.hand_detector:main',
            'object_detector = handover_vision.object_detector:main',
            'kalman_filter_node = handover_vision.kalman_filter_node:main',
            'pointcloud_merger = handover_vision.pointcloud_merger:main',
        ],
    },
)

