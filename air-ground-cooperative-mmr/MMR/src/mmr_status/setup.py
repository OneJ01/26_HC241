from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mmr_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('mmr_status/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='onej-01',
    maintainer_email='onej-01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mmr_status_node = mmr_status.mmr_status_node:main',
            'tilt_monitor_node = mmr_status.tilt_monitor_node:main',
        ],
    },
)
