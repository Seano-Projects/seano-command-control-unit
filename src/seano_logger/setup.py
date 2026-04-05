from setuptools import find_packages, setup

package_name = 'seano_logger'

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
    maintainer_email='seano@todo.todo',
    description='CSV logger untuk semua data yang dikirim ke MQTT',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'csv_logger_node = seano_logger.csv_logger_node:main',
        ],
    },
)
