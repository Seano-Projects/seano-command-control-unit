from setuptools import find_packages, setup

package_name = 'seano_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/command.launch.py']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='seano',
    maintainer_email='seano@todo.todo',
    description='SEANO Command receiver from MQTT',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_node = seano_command.command_node:main',
            'waypoint_node = seano_command.waypoint_node:main',
            'thruster_node = seano_command.thruster_node:main',
        ],
    },
)
