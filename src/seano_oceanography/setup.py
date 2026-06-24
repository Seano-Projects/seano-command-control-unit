from setuptools import find_packages, setup

package_name = 'seano_oceanography'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seano',
    maintainer_email='seano@todo.todo',
    description='Oceanography sensor nodes for CTD and ADCP.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ctd_sensor_node = seano_oceanography.ctd_sensor_node:main',
            'adcp_sensor_node = seano_oceanography.adcp_sensor_node:main',
        ],
    },
)
