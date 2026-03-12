from setuptools import find_packages, setup

package_name = 'seano_cam'

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
    description='SEANO Camera USB detection and future AI capabilities',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = seano_cam.camera_node:main',
            'camera_viewer = seano_cam.camera_viewer:main',
            'rtmp_streamer = seano_cam.rtmp_streamer:main',
        ],
    },
)
