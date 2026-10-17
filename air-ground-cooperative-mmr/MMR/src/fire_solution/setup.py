from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fire_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyproj'],
    zip_safe=True,
    maintainer='onej-01',
    maintainer_email='onej-01@todo.todo',
    description='포신 앙각/방위각 계산 노드 (DEM 지형 보정 포함)',
    license='Apache License 2.0',
    extras_require={'test': ['pytest']},
	entry_points={
	    'console_scripts': [
		'fire_solution_node = fire_solution.fire_solution_node_completed:main',
		'turret_control_node = fire_solution.turret_control_node:main',
		'aim_coordinator_node = fire_solution.aim_coordinator_node:main',
	    ],
	},
)
