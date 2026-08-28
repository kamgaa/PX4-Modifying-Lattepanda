import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'px4_mocap_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SEOSUK',
    maintainer_email='seosuk@users.noreply.github.com',
    description=(
        'Bridge named ROS 2 motion-capture poses to PX4 external-vision '
        'odometry.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_bridge_node = px4_mocap_bridge.mocap_bridge_node:main',
        ],
    },
)
