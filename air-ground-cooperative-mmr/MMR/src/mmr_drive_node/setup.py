from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mmr_drive_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='onej',
    maintainer_email='todo@todo.com',
    description='MMR BLDC 6-wheel drive node via BLC-400R4E RS-485',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bldc_drive_node = mmr_drive_node.bldc_drive_node:main',
            'keyboard_joy_node = mmr_drive_node.keyboard_joy_node:main',
        ],
    },
)
