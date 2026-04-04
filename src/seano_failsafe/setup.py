from setuptools import find_packages, setup

package_name = 'seano_failsafe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seano',
    maintainer_email='dzikriibnuf@gmail.com',
    description='SEANO Failsafe System - Battery monitoring and emergency procedures',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'seano_battery = seano_failsafe.seano_battery:main',
            'seano_battery_dummy = seano_failsafe.seano_battery_dummy:main',
            'seano_failsafe = seano_failsafe.seano_failsafe:main',
            'seano_communication_monitor = seano_failsafe.seano_communication_monitor:main',
        ],
    },
)
