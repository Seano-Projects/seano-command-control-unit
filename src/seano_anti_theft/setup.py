from setuptools import setup
import os
from glob import glob

package_name = 'seano_anti_theft'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seano',
    maintainer_email='seano@example.com',
    description='ROS2 anti-theft node for CUAV / ArduPilot telemetry monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anti_theft_node = seano_anti_theft.anti_theft_node:main',
        ],
    },
)