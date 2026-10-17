from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mmr_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt6', 'pyproj', 'numpy'],
    zip_safe=True,
    maintainer='onej-01',
    maintainer_email='onej-01@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'main_gui = mmr_gui.main_gui:main',
            'drive_debug_gui = mmr_gui.drive_debug_gui:main',
        ],
    },
)
