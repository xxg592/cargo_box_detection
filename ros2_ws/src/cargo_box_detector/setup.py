from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'cargo_box_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxg592',
    maintainer_email='xxg592@todo.todo',
    description='ROS 2 cargo box detection, delivery FSM and metrics',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cargo_box_detector_node = cargo_box_detector.detector_node:main',
            'cargo_delivery_fsm_node = cargo_box_detector.fsm_node:main',
            'cargo_metrics_node = cargo_box_detector.metrics_node:main',
            'cargo_position_fusion_node = cargo_box_detector.fusion_node:main',
            'cargo_navigation_node = cargo_box_detector.navigation_node:main',
        ],
    },
)
