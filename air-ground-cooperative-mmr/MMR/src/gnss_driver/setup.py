from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gnss_driver'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dohwanko',
    maintainer_email='dohwanko@todo.todo',
    description='u-blox ZED-F9P 듀얼 안테나 GNSS 드라이버',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ubx_rover = gnss_driver.ubx_rover_node:main',
            'ntrip_lite = gnss_driver.ntrip_lite_node:main',
            'manual_target = gnss_driver.manual_distance_input:main',
        ],
    },
)
